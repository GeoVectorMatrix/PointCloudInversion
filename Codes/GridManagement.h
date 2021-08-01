#pragma once
using namespace std;
#include <vector>

//////////////////////////////////////////////////////////////////////////
template <typename T>
class CSquare2D
{
  public:
	CSquare2D(void);
	~CSquare2D(void);
  public:
	T    minZ;           
	T    maxZ;           
	T    maxzX;          
	T    maxzY;          
	T    minzY;        
	T    minzX;        
	//
	T    Recxmin;        
	T    Recymin;
	T    Recxmax;        
	T    Recymax;
	vector<unsigned int>      IdArray;     
};

template <typename T> 
CSquare2D<T>::CSquare2D(void)
{
	minZ=1000000.0;
	maxZ=-1000000.0;
	minzX=0;
	minzY=0;
	maxzX =0;           
	maxzY =0;           
	//
	Recxmin=0;         
	Recymin=0;
	Recxmax=0;        
	Recymax=0;
	//
	IdArray.clear();
}

template <typename T> 
CSquare2D<T>::~CSquare2D(void)
{
}

//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT>
class CGridManagement
{
public:
	CGridManagement(void);
	~CGridManagement(void);
	CSquare2D<T>      *Square2D;
public:
	T _grid_length;
	T _grid_width;
	unsigned int    globleRow;                   
	unsigned int    globleColum;                 
	bool   is_divided;  

public:
	void Do_Grid(const vector<PointT> &pt,const vector<unsigned int> &ids); 
	void Do_Grid(const vector<PointT> &pt);  
public:
	unsigned int           GetCellOfXY(T X,T Y);                     
	vector<unsigned int>   GetCellOfXY(T X,T Y,unsigned int r_grid);  
	vector<unsigned int>   GetCellOfXY(T X,T Y,T r_grid); 
	unsigned int getindexOfij(unsigned int I, unsigned int J);
	int          getindexOfijSelf(unsigned int I,unsigned int J); // Boundary warning

private:
	T org_x;    T org_y; 
	T  top_x;   T top_y;
private:
	void translate(const vector<PointT> &pt); 
	void updataSquare(CSquare2D<T> &needUpdata,PointT  ptup);
	void getijfromxy(T X,T Y,unsigned int &i,unsigned int &j);

};

//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> 
CGridManagement<typename T, typename PointT>::CGridManagement(void)
{
	_grid_length=0.5;
	_grid_width=_grid_length;
	//
	org_x=0;    org_y=0; 
	top_x=0;    top_y=0;
	//
	globleRow=0;  
	globleColum=0;
	//
	is_divided=false;
	//
	Square2D=NULL;

}
template <typename T, typename PointT>
CGridManagement<typename T, typename PointT>::~CGridManagement(void)
{
	if (Square2D!=NULL)
	{
		delete[] Square2D; 
	}


}

//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> void
CGridManagement<typename T, typename PointT>::Do_Grid(const vector<PointT> &pt,const vector<unsigned int> &ids)
{
	if (pt.size()<0)
	{
		return;
	}
	translate(pt);

	if (top_x<top_y)
	{
		T Tempdata=0;
		Tempdata = _grid_width;
		_grid_width=_grid_length;
		_grid_length=Tempdata;
	}
	globleColum  =  unsigned int( ceil(top_x/_grid_length) ); //x===col
	globleRow    =  unsigned int( ceil(top_y/_grid_width) );  //y==row
	if(globleColum==0||globleRow==0)
	{
		return; 
	}
	Square2D=new CSquare2D<T>[globleRow*globleColum];
	unsigned int tmi=0;   unsigned int tmj=0;
	for (unsigned int i=0;i<pt.size();i++)
	{
		tmj=unsigned int(floor((pt[i].x-org_x)/_grid_length));
		tmi=unsigned int(floor((pt[i].y-org_y)/_grid_width));
		Square2D[tmi*globleColum+tmj].IdArray.push_back(ids[i]);
		updataSquare(Square2D[tmi*globleColum+tmj],pt[i]);
	}
	for (unsigned int i=0;i<globleRow;i++)
	{
		for (unsigned int j=0;j<globleColum;j++)
		{
			Square2D[i*globleColum+j].Recxmin=org_x+j*_grid_length;
			Square2D[i*globleColum+j].Recxmax=org_x+(j+1)*_grid_length;
			Square2D[i*globleColum+j].Recymin=org_y+i*_grid_length;
			Square2D[i*globleColum+j].Recymax=org_y+(i+1)*_grid_length;
		}
	}
	is_divided=true;
}
//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> void
CGridManagement<typename T, typename PointT>::Do_Grid(const vector<PointT> &pt)
{
	if (pt.size()<0)
	{
		return;
	}
	translate(pt);
	if (top_x<top_y)
	{
		T Tempdata=0;
		Tempdata=_grid_width;
		_grid_width=_grid_length;
		_grid_length=Tempdata;
	}
	globleColum  =  unsigned int( ceil(top_x/_grid_length) ); //x===col
	globleRow    =  unsigned int( ceil(top_y/_grid_width) );  //y==row
	if(globleColum==0||globleRow==0)
	{
		return; 
	}
	Square2D=new CSquare2D<T>[globleRow*globleColum];
	unsigned int tmi=0;   unsigned int tmj=0;
	for (unsigned int i=0;i<pt.size();i++)
	{
		tmj=unsigned int(floor((pt[i].x-org_x)/_grid_length));
		tmi=unsigned int(floor((pt[i].y-org_y)/_grid_width));
		Square2D[tmi*globleColum+tmj].IdArray.push_back(i);
		updataSquare(Square2D[tmi*globleColum+tmj],pt[i]);
	}
	for (unsigned int i=0;i<globleRow;i++)
	{
		for (unsigned int j=0;j<globleColum;j++)
		{
			Square2D[i*globleColum+j].Recxmin=org_x+j*_grid_length;
			Square2D[i*globleColum+j].Recxmax=org_x+(j+1)*_grid_length;
			Square2D[i*globleColum+j].Recymin=org_y+i*_grid_length;
			Square2D[i*globleColum+j].Recymax=org_y+(i+1)*_grid_length;
		}
	}
	is_divided=true;
}

//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> void
CGridManagement<typename T, typename PointT>::updataSquare(CSquare2D<T> &needUpdata,PointT  ptup)
{
	if (ptup.z>needUpdata.maxZ)
	{
		needUpdata.maxZ  =  ptup.z;
		needUpdata.maxzX =  ptup.x;
		needUpdata.maxzY =  ptup.y;
	}
	if (ptup.z<needUpdata.minZ)
	{
		needUpdata.minZ = ptup.z;
		needUpdata.minzX= ptup.x;
		needUpdata.minzY= ptup.y;
	}
}

template <typename T, typename PointT> void
CGridManagement<typename T, typename PointT>::translate(const vector<PointT> &pt)
{
	T MINX=pt[0].x;
	T MINY=pt[0].y;
	T MaxX=pt[0].x;
	T MaxY=pt[0].y;
	for (int i=1;i<pt.size();++i)
	{
		if (pt[i].x<MINX)
		{
			MINX=pt[i].x;
		}
		if (pt[i].y<MINY)
		{
			MINY=pt[i].y;
		}
		if (pt[i].x>MaxX)
		{
			MaxX=pt[i].x;
		}
		if (pt[i].y>MaxY)
		{
			MaxY=pt[i].y;
		}
	}
	org_x=MINX;
	org_y=MINY;
	//
	top_x=MaxX-MINX+0.0001;
	top_y=MaxY-MINY+0.0001;  
}
template <typename T, typename PointT> unsigned int
CGridManagement<typename T, typename PointT>::getindexOfij(unsigned int I,unsigned int J)
{
	//If out of range, return boundary index
	if (I > globleRow - 1)
	{
		I = globleRow - 1;
	}
	if (I < 0)
	{
		I = 0;
	}
	/////
	if (J > globleColum - 1)
	{
		J = globleColum - 1;
	}
	if (J < 0)
	{
		J = 0;
	}
	return I*globleColum+J;	
}
template <typename T, typename PointT>  int
CGridManagement<typename T, typename PointT>::getindexOfijSelf(unsigned int I, unsigned int J)
{
	//If out of range, return boundary index
	if (I > globleRow - 1)
	{
		return -1;
	}
	if (I < 0)
	{
		return -1;
	}
	/////
	if (J > globleColum - 1)
	{
		return -1;
	}
	if (J < 0)
	{
		return -1;
	}
	return I*globleColum + J;
}


//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> void
CGridManagement<typename T, typename PointT>:: getijfromxy(T X,T Y,unsigned int &i,unsigned int &j)
{
	i=unsigned int(floor((Y-org_y)/_grid_width));
	j=unsigned int(floor((X-org_x)/_grid_length));
	if ((X-org_x)>top_x)
	{
		j=unsigned int(floor(top_x/_grid_length));
	}
	if ((Y-org_y)>top_y)
	{
		i=unsigned int(floor(top_y/_grid_width));
	}
	if ((X-org_x)<0)
	{
		j=0;
	}
	if((Y-org_y)<0)
	{
		i=0;
	}
}


template <typename T, typename PointT> 	unsigned int
CGridManagement<typename T, typename PointT>:: GetCellOfXY(T X,T Y)
{
	unsigned int ix; unsigned int iy;
	getijfromxy(X,Y,ix,iy);
	return(getindexOfij(ix,iy));
}

template <typename T, typename PointT> vector<unsigned int>   
CGridManagement<typename T, typename PointT>::GetCellOfXY(T X,T Y,unsigned int r_grid)
{
	vector<unsigned int>  ids;
	//
	unsigned int the_ID=GetCellOfXY(X,Y); 
	//
	unsigned int ix=0;
	unsigned int iy=0;
	getijfromxy(X,Y,ix,iy);
	unsigned int ix_temp=0;
	unsigned int iy_temp=0;
	for (unsigned int k=0;k<r_grid;k++ )
	{
		ix_temp=ix-k;
		if (ix_temp<0)
		{
			break;
		}
		for (unsigned int t=0;t<r_grid;t++ )
		{
			iy_temp=iy-t;
		   if (iy_temp<0)
		   {
			   break;
		   }
		   ids.push_back(getindexOfij(ix_temp,iy_temp));
		}
	}
	//
	for (unsigned int k=1;k<r_grid;k++ )
	{
		ix_temp=ix+k;
		if (ix_temp>floor(top_x/_grid_length))
		{
			break;
		}
		for (unsigned int t=0;t<r_grid;t++ )
		{
			iy_temp=iy+t;
			if (iy_temp>floor(top_y/_grid_width))
			{
				break;
			}
			ids.push_back(getindexOfij(ix_temp,iy_temp));
		}
	}
	return ids;
}

template <typename T, typename PointT> vector<unsigned int>   
CGridManagement<typename T, typename PointT>:: GetCellOfXY(T X,T Y,T r_grid)
{
	T leftupPointX=ptIN.x-r_grid;
	T leftupPointY=ptIN.y-r_grid;
	T rightPointX =ptIN.x+r_grid;
	T rightPointY =ptIN.y+r_grid;
	unsigned int imax,jmax,imin,jmin;

	getijfromxy(leftupPointX,leftupPointY,  imin, jmin);
	getijfromxy(rightPointX,  rightPointY,  imax, jmax);

	vector<unsigned int>  tempSave;

	for (unsigned int i=imin;i<=imax;i++)
	{
		for (unsigned int j=jmin;j<=jmax;j++)
		{
			for (unsigned  t=0;t<Square2D[i*globleColum+j].IdArray.size();t++)
			{
				tempSave.push_back(Square2D[i*globleColum+j].IdArray[t]);
			}
		}
	}
	return tempSave;

}
