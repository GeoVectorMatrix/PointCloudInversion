/*
Please cite 
"Shaobo Xia, Dong Chen, Jiju Peethambaran, Pu Wang and Sheng Xu*; Point Cloud Inversion: A Novel Approach for the Localization of Trees in Forests from TLS Data, Remote Sensing."
if this idea helps in your research.
//////////
- This file containts the origninal C++ code for PCI (How to inverse point clouds and How to find loacal maxmia)
- How to read/use the PCI code:
   CInverseTops<Point3f>    TreeTops;   // New a class
   TreeTops.VoxSize = 0.25;             // Set voxel resolution 0.25m
   TreeTops.SetPt(pDoc->m_vPoint._pTC); // Set input point clouds
   TreeTops.GetColumnWiseEmpty();       // Caculate the empty index, Eq.(1)
   TreeTops.GetInversedZ();             // Run-PCI, Eq.(2)
//////////
*/
#pragma once
using namespace std;
#include "VoxelManagement.h"
#include "GridManagement.h"
/* 
For GridManagement.h and GridManagement.h: 
These two classes are not public available now as they are owned by others. We will release them after rewriting.
The function of these two classes is data managememt, VoxelManagement.h for 3D voxel structures and GridManagement.h for 2D grid structures.
They are not a part of PCI.
They are easy to understand, and can be easily replaced by other data managememt implementations.
Researchers who are interested in the whole program are encouraged to contact authors for more information.
*/
#include <vector>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>

template <typename PointT>
class CInverseTops
{
public:
	CInverseTops();
	~CInverseTops();
	void                               SetPt(vector<PointT> &pt); // PointT should contain (x,y,z) elements.
	CVoxelManagement<float, PointT>    Vox;   //3D voxel managememt
	CGridManagement<float, PointT>     Grids; //2D grid management
	int                                VoxLength;
	vector<PointT>                     TopLocations;
public:
	float                              MinTreeH;
	float                              NomaxDist;
	vector<float>                      ColumnEmpty;
	float                              VoxSize;
	vector<PointT>                     *_Pc;             //pointer to original point clouds
public:
	void                               GetColumnWiseEmpty();
	void                               GetInversedZ();
	void                               GetTopCandidates();
	void                               NoMaxFilter();
public:
	void                               Save2DStemsTXT(string filename);
	void                               SetShift(double sx, double sy, double sz);
	double                             shiftX;  // Used in data saving
	double                             shiftY;
	double                             shiftZ;
private:
	void                               _VoxGenerate();
	float                              _p2p2DDist(PointT pA, PointT pB);
};

template <typename PointT>
CInverseTops<PointT>::CInverseTops()
{
	VoxSize    = 0.25; // Grid/Vox size
	VoxLength  = 0;
	_Pc        = NULL;
	/////////////////////////////////// Parameters for specific applications.
	MinTreeH   = 3.0; // Used in specific scenes (Remove noise/Outlier after finding local maxima,NOT a part of PCI)
	NomaxDist  = 2.0; // Used in specific scenes (Remove noise/Outlier after finding local maxima,NOT a part of PCI)
}


/**A part of PCI: Get Column-wise empty index,ref- Eq(1)**/
template <typename PointT> void
CInverseTops<PointT>::GetColumnWiseEmpty()
{
	_VoxGenerate(); // the voxel management 
	//
	ColumnEmpty.clear();
	ColumnEmpty.resize(Vox.globleRow*Vox.globleColum,0.0);
	for (unsigned int i = 0; i < Vox.globleRow; i++)
	{
		for (unsigned int j = 0; j < Vox.globleColum; j++)
		{
			for (unsigned int k = 0; k < Vox.globleZHeight; k++)
			{
				// More than One point (In case of noise)
				if (Vox.Square3D[k*Vox.globleColum*Vox.globleRow + i*Vox.globleColum + j].IdArray.size() < 2)
				{
					ColumnEmpty[i*Vox.globleColum + j] = ColumnEmpty[i*Vox.globleColum + j] + VoxSize;
				}
			}
		}
	}
}

/**A part of PCI: Get point clouds after using PCI, ref- Eq(2)**/
template <typename PointT> void
CInverseTops<PointT>::GetInversedZ()
{
	float maxZ = Vox.globleZHeight*VoxSize; // the max Z will be used a scale
	for (unsigned int i = 0; i < Vox.globleRow; i++)
	{
		for (unsigned int j = 0; j < Vox.globleColum; j++)
		{
			float dzColumn = ColumnEmpty[i*Vox.globleColum + j];
			for (unsigned int k = 0; k < Vox.globleZHeight; k++)
			{
				for (unsigned int t = 0; t < Vox.Square3D[k*Vox.globleColum*Vox.globleRow + i*Vox.globleColum + j].IdArray.size(); t++)
				{
					// Get the point ID for each member in the voxel which is indexed by [k*Vox.globleColum*Vox.globleRow + i*Vox.globleColum + j]
					unsigned int theID =  Vox.Square3D[k*Vox.globleColum*Vox.globleRow + i*Vox.globleColum + j].IdArray[t];
					_Pc->at(theID).z =  max(maxZ - dzColumn - _Pc->at(theID).z, 0.0);
					// At this point, the input data have been enhanced by PCI, they are saved in _Pc
				}
			}
		}
	}
}

/**This is an exmple of the Top-based methods, NOT a part of PCI **/
// In this version, the top-based method is based on 2D grids. 
// However, this can also be done in 3D voxels. 
template <typename PointT> void
CInverseTops<PointT>::GetTopCandidates()
{
	Grids._grid_length = VoxSize;
	Grids._grid_width  = VoxSize;
	Grids.Do_Grid(*_Pc); 
	TopLocations.clear();
	// A way to find loacl max
	for (unsigned int i = 0; i < Grids.globleRow; i++)
	{
		for (unsigned int j = 0; j < Grids.globleColum; j++)
		{
			unsigned int theID = Grids.getindexOfij(i, j);
			if (Grids.Square2D[theID].IdArray.size() < 1)
				continue;// Empty grid
			float centerZ = Grids.Square2D[theID].maxZ;
			if (centerZ < MinTreeH)
			{
				continue; // Remove 'low/small' tops for specific applications. 
				// MinTreeH could set to zero if we do not need this filter in general cases. 
			}
			/****************************************************/
			// 8-neighbours
			unsigned int ThisNeighbour = Grids.getindexOfij(i - 1, j - 1); // Get neighbours
			float thisZ = Grids.Square2D[ThisNeighbour].maxZ;
			if (thisZ > centerZ)
			{
				continue;
			}
			ThisNeighbour = Grids.getindexOfij(i - 1, j);// Get neighbours
			thisZ = Grids.Square2D[ThisNeighbour].maxZ;
			if (thisZ > centerZ)
			{
				continue;
			}
			ThisNeighbour = Grids.getindexOfij(i - 1, j + 1);// Get neighbours
			thisZ = Grids.Square2D[ThisNeighbour].maxZ;
			if (thisZ > centerZ)
			{
				continue;
			}
			/////
			ThisNeighbour = Grids.getindexOfij(i, j - 1);// Get neighbours
			thisZ = Grids.Square2D[ThisNeighbour].maxZ;
			if (thisZ > centerZ)
			{
				continue;
			}
			ThisNeighbour = Grids.getindexOfij(i, j + 1);// Get neighbours
			thisZ = Grids.Square2D[ThisNeighbour].maxZ;
			if (thisZ > centerZ)
			{
				continue;
			}
			///////
			ThisNeighbour = Grids.getindexOfij(i + 1, j - 1);// Get neighbours
			thisZ = Grids.Square2D[ThisNeighbour].maxZ;
			if (thisZ > centerZ)
			{
				continue;
			}
			ThisNeighbour = Grids.getindexOfij(i + 1, j);// Get neighbours
			thisZ = Grids.Square2D[ThisNeighbour].maxZ;
			if (thisZ > centerZ)
			{
				continue;
			}
			ThisNeighbour = Grids.getindexOfij(i + 1, j + 1);// Get neighbours
			thisZ = Grids.Square2D[ThisNeighbour].maxZ;
			if (thisZ > centerZ)
			{
				continue;
			}
			/*********************************************/
			PointT thisCenter;
			thisCenter.x = 0.5*(Grids.Square2D[theID].Recxmin + Grids.Square2D[theID].Recxmax);
			thisCenter.y = 0.5*(Grids.Square2D[theID].Recymin + Grids.Square2D[theID].Recymax); //Grid-wise local-max
			//thisCenter.x = Grids.Square2D[theID].maxzX;
			//thisCenter.y = Grids.Square2D[theID].maxzY; //An alternative way; Point-wise local-max
			thisCenter.z = centerZ;
			TopLocations.push_back(thisCenter);
		}
	}
}


/**This can be used in post-processing, NOT a part of PCI**/
template <typename PointT> void
CInverseTops<PointT>::NoMaxFilter()
{
	vector<bool> IsMax;
	IsMax.resize(TopLocations.size(), true);
	for (unsigned int i = 0; i < TopLocations.size()-1; i++)
	{
		for (unsigned int j = i+1; j < TopLocations.size(); j++)
		{
			float thisD = _p2p2DDist(TopLocations[i], TopLocations[j]);
			if (thisD < NomaxDist) // Too close to each other.
			{
				if (TopLocations[i].z > TopLocations[j].z)
				{
					IsMax[j] = false;
				}
				else {IsMax[i] = false; break; }// Stop this loop 
			}
		}
	}
	//
	vector<PointT>        TopLocations_Copy;
	for (unsigned int i = 0; i < IsMax.size(); i++)
	{
		if (IsMax[i])
		{
			TopLocations_Copy.push_back(TopLocations[i]);
		}
	}
	//
	TopLocations.clear();
	TopLocations = TopLocations_Copy;
}

/********************************************************************/
/**Save top locations to files**/
template <typename PointT> void 
CInverseTops<PointT>::Save2DStemsTXT(string filename)
{
	string filenameC = filename;
	size_t found;
	found = filenameC.find_last_of("/.");
	string fold = filename.substr(0, found);
	int l = filenameC.length();
	string fileFeature;
	fileFeature = fold + ".stem"; // The final file. 
	ofstream ofileline;
	ofileline.open(fileFeature);
	for (unsigned int i = 0; i < TopLocations.size(); i++)
	{   
		ofileline << i+1 << " "
        		  << setprecision(15) << double(TopLocations[i].x + shiftX) << " "
			  << setprecision(15) << double(TopLocations[i].y + shiftY) << " "
		          << setprecision(15) << double(TopLocations[i].z + shiftZ) << " "
		          << 0.0 << endl; //dbh, not available here
	}
	ofileline.close();
}

template <typename PointT> float
CInverseTops<PointT>::_p2p2DDist(PointT pA, PointT pB)
{
	return sqrt(pow(pA.x - pB.x, 2.0) + pow(pA.y - pB.y, 2.0));
}

template <typename PointT>
CInverseTops<PointT>::~CInverseTops()
{
	_Pc = NULL;
}
//
template <typename PointT> void
CInverseTops<PointT>::SetPt(vector<PointT> &pt)
{
	_Pc = NULL;
	_Pc = &pt;

}
template <typename PointT> void 
CInverseTops<PointT>::SetShift(double sx, double sy, double sz)
{
	shiftX = sx;
	shiftY = sy;
	shiftZ = sz;
}
template <typename PointT> void
CInverseTops<PointT>::_VoxGenerate()
{
	Vox._cell_size = VoxSize;
	Vox.Do_Grid(*_Pc);
	VoxLength = Vox.globleZHeight*Vox.globleRow*Vox.globleColum;
}
