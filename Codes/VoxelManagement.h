#pragma once
using namespace std;
#include <vector>

//////////////////////////////////////////////////////////////////////////
template <typename T>
class CSquare3D
{
  public:
	CSquare3D(void);
	~CSquare3D(void);
  public:
	 T        RecXmin;       
	 T        RecYmin;
	 T        RecZmin;
	 T        RecXmax;        
	 T        RecYmax;
	 T        RecZmax;
	vector<unsigned int>      IdArray;       //一个区域的所有数据点的ID  数据最大
};

template <typename T> 
CSquare3D<T>::CSquare3D(void)
{
	RecXmin=0;       
	RecYmin=0;
	RecZmin=0;
	RecXmax=0;        
	RecYmax=0;
	RecZmax=0;
	//
	IdArray.clear();
}

template <typename T> 
CSquare3D<T>::~CSquare3D(void)
{
}

//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT>
class CVoxelManagement
{
public:
	CVoxelManagement(void);
	~CVoxelManagement(void);
	CSquare3D<T>      *Square3D;
public:
	T                    _cell_size;
	unsigned int        globleRow;                    //行数
	unsigned int        globleColum;                  //列数目
	unsigned int        globleZHeight;                //高程格网数目
	bool                is_divided;  //是否分割

public:
	void Do_Grid(const vector<PointT> &pt,const vector<unsigned int> &ids); //需要输入每个点特定的iD
	void Do_Grid(const vector<PointT> &pt);  //默认ID为序号
public:
	unsigned int           GetCellOfXYZ(T X,T Y,T Z);                      //输入，xyZ，获得对应格网的数组位置
	vector<unsigned int>   GetCellOfXYZ(T X,T Y,T Z,unsigned int r_grid);  //输入xyZ 获得所在格网，以及格网周围r个格网的 数组索引
	vector<unsigned int>   GetCellOfXYZ(T X,T Y,T Z,T r_grid);             //输入xyZ 获得所在格网，以及该点周围，半径为r的所有格网点 数组索引
	//
	/************************************************************************/
	/*       由 ij  J得到的是数组中的索引                                     */
	/************************************************************************/
	unsigned int getindexOfijf(unsigned int I, unsigned int J, unsigned int F);
	void         getIJF(unsigned int &I,unsigned int &J,unsigned int &F, unsigned int longIdx);
public:
	T org_x;    T org_y; T org_z; 
	T  top_x;   T top_y; T top_z;
private:
	//设置边界 保证对最小空间进行分割
	void translate(const vector<PointT> &pt); 
	//
	/************************************************************************/
	/* 给出坐标x y     获得所在的数据块  i  j                               */
	/************************************************************************/
	void getijffromxyz(T X,T Y,T Z,unsigned int &i,unsigned int &j,unsigned int &f);

};

//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> 
CVoxelManagement<typename T, typename PointT>::CVoxelManagement(void)
{
	_cell_size=0.5;
	//
	org_x=0;    org_y=0;    org_z=0; 
	top_x=0;    top_y=0;    top_z=0;
	//
	globleRow=0;  
	globleColum=0;
	globleZHeight=0;
	//
	is_divided=false;
	//
	Square3D=NULL;

}
template <typename T, typename PointT>
CVoxelManagement<typename T, typename PointT>::~CVoxelManagement(void)
{
	if (Square3D!=NULL)
	{
		delete[] Square3D; 
	}
}

//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> void
CVoxelManagement<typename T, typename PointT>::Do_Grid(const vector<PointT> &pt,const vector<unsigned int> &ids)
{
	if (pt.size()<0)
	{
		return;
	}
	translate(pt);// 找边界
	//获得分割的结果  x  y方向分割数目  即  行数 列数 
	globleColum  =  unsigned int( ceil(top_x/_cell_size) ); //x===col
	globleRow    =  unsigned int( ceil(top_y/_cell_size) );  //y==row
	globleZHeight =  unsigned int( ceil(top_z/_cell_size) );
	if(globleColum==0||globleRow==0||globleZHeight==0)
	{
		return; //如果输入的间隔比点云的范围还大了，显然不能做格网了
	}
	//获得数组的大小
	Square3D=new CSquare3D<T>[globleRow*globleColum*globleZHeight];
	//进行数据分割
	unsigned int tmi=0;   unsigned int tmj=0;	unsigned int tmt=0;
	for (unsigned int i=0;i<pt.size();i++)
	{
		tmj=unsigned int(floor((pt[i].x-org_x)/_cell_size));
		tmi=unsigned int(floor((pt[i].y-org_y)/_cell_size));
		tmt=unsigned int(floor((pt[i].z-org_z)/_cell_size));
		Square3D[tmt*globleColum*globleRow+tmi*globleColum+tmj].IdArray.push_back(ids[i]);
	}
	//格网边界
	for (unsigned int t=0;t<globleZHeight;t++)
	{
		for (unsigned int i=0;i<globleRow;i++)
		{
			for (unsigned int j=0;j<globleColum;j++)
			{
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecZmin=org_z+t*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecZmax=org_z+(t+1)*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecXmin=org_x+j*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecXmax=org_x+(j+1)*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecYmin=org_y+i*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecYmax=org_y+(i+1)*_cell_size;
			}	
		}		
	}
	is_divided=true;
}
//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> void
CVoxelManagement<typename T, typename PointT>::Do_Grid(const vector<PointT> &pt)
{
	if (pt.size()<0)
	{
		return;
	}
	translate(pt);// 找边界
	//获得分割的结果  x  y方向分割数目  即  行数 列数 
	globleColum  =  unsigned int( ceil(top_x/_cell_size) ); //x===col
	globleRow    =  unsigned int( ceil(top_y/_cell_size) );  //y==row
	globleZHeight =  unsigned int( ceil(top_z/_cell_size) );
	if(globleColum==0||globleRow==0||globleZHeight==0)
	{
		return; //如果输入的间隔比点云的范围还大了，显然不能做格网了
	}
	//获得数组的大小
	Square3D=new CSquare3D<T>[globleRow*globleColum*globleZHeight];
	//进行数据分割
	unsigned int tmi=0;   unsigned int tmj=0;	unsigned int tmt=0;
	for (unsigned int i=0;i<pt.size();i++)
	{
		tmj=unsigned int(floor((pt[i].x-org_x)/_cell_size));
		tmi=unsigned int(floor((pt[i].y-org_y)/_cell_size));
		tmt=unsigned int(floor((pt[i].z-org_z)/_cell_size));
		Square3D[tmt*globleColum*globleRow+tmi*globleColum+tmj].IdArray.push_back(i);
	}
	//格网边界
	for (unsigned int t=0;t<globleZHeight;t++)
	{
		for (unsigned int i=0;i<globleRow;i++)
		{
			for (unsigned int j=0;j<globleColum;j++)
			{
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecZmin=org_z+t*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecZmax=org_z+(t+1)*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecXmin=org_x+j*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecXmax=org_x+(j+1)*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecYmin=org_y+i*_cell_size;
				Square3D[t*globleColum*globleRow+i*globleColum+j].RecYmax=org_y+(i+1)*_cell_size;
			}	
		}		
	}
	is_divided=true;
}

//////////////////////////////////////////////////////////////////////////
////设置边界 保证对最小空间进行分割
template <typename T, typename PointT> void
CVoxelManagement<typename T, typename PointT>::translate(const vector<PointT> &pt)
{
	T MINX=pt[0].x;
	T MINY=pt[0].y;
	T MINZ=pt[0].z;
	T MaxX=pt[0].x;
	T MaxY=pt[0].y;
	T MaxZ=pt[0].z;
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
		if (pt[i].z<MINZ)
		{
			MINZ=pt[i].z;
		}

		if (pt[i].x>MaxX)
		{
			MaxX=pt[i].x;
		}
		if (pt[i].y>MaxY)
		{
			MaxY=pt[i].y;
		}
		if (pt[i].z>MaxZ)
		{
			MaxZ=pt[i].z;
		}
	}
	org_x=MINX;
	org_y=MINY;
	org_z=MINZ;
	//
	top_x=MaxX-MINX+0.0001;
	top_y=MaxY-MINY+0.0001;  //增加一个扰动，确保分割稳定性
	top_z=MaxZ-MINZ+0.0001;  //增加一个扰动，确保分割稳定性
}

//////////////////////////////////////////////////////////////////////////
////设置边界 保证对最小空间进行分割
template <typename T, typename PointT> unsigned int
CVoxelManagement<typename T, typename PointT>::getindexOfijf(unsigned int I,unsigned int J,unsigned int F)
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
	//
	if (F > globleZHeight - 1)
	{
		F = globleZHeight - 1;
	}
	if (F < 0)
	{
		F = 0;
	}
	//
	unsigned int theID = F*globleColum*globleRow + I*globleColum + J;
	return  theID;
}

template <typename T, typename PointT> void
CVoxelManagement<typename T, typename PointT>::getIJF(unsigned int &I, unsigned int &J, unsigned int &F, unsigned int longIdx)
{
	if (longIdx > globleRow*globleColum*globleZHeight - 1)
	{
		longIdx = globleRow*globleColum*globleZHeight - 1;
	}
	F = unsigned int (floor(double(longIdx) / double(globleRow*globleColum)));
	double rm = double(longIdx) - double(F*globleRow*globleColum);
	I = unsigned int (floor(rm / double(globleColum)));
	//
	J = int(rm - double(I*globleColum));
}
//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> void
CVoxelManagement<typename T, typename PointT>:: getijffromxyz(T X,T Y,T Z,unsigned int &i,unsigned int &j,unsigned int &f)
{
	i=unsigned int(floor((Y-org_y)/_cell_size));
	j=unsigned int(floor((X-org_x)/_cell_size));
	f=unsigned int(floor((Z-org_z)/_cell_size));
	// 下面防止代码奔溃，做了强制的范围约束，事实上，自己用不会出现这些情况
	if ((X-org_x)>top_x)
	{
		j=unsigned int(floor(top_x/_cell_size));
	}
	if ((Y-org_y)>top_y)
	{
		i=unsigned int(floor(top_y/_cell_size));
	}
	if ((Z-org_z)>top_z)
	{
		f=unsigned int(floor(top_z/_cell_size));
	}

	if ((X-org_x)<0)
	{
		j=0;
	}
	if((Y-org_y)<0)
	{
		i=0;
	}
	if((Z-org_z)<0)
	{
		f=0;
	}
}

//////////////////////////////////////////////////////////////////////////
// 为外部 访问格网 提供接口 
//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> 	unsigned int
CVoxelManagement<typename T, typename PointT>:: GetCellOfXYZ(T X,T Y,T Z)
{
	unsigned int ix; unsigned int iy;unsigned int i_f;
	getijffromxyz(X,Y,Z,ix,iy,i_f);
	return(getindexOfijf(ix,iy,i_f));
}
//////////////////////////////////////////////////////////////////////////
////输入xy 获得所在格网，以及格网周围r个格网的 数组索引
//***
template <typename T, typename PointT> vector<unsigned int>   
CVoxelManagement<typename T, typename PointT>::GetCellOfXYZ(T X,T Y,T Z,unsigned int r_grid)
{
	vector<unsigned int>  ids;
	//
	unsigned int the_ID=GetCellOfXYZ(X,Y,Z); //该点自身的id
	//
	unsigned int ix=0;
	unsigned int iy=0;
	unsigned int i_f=0;
	getijffromxyz(X,Y,Z,ix,iy,i_f);
	unsigned int ix_temp=0;
	unsigned int iy_temp=0;
	unsigned int if_temp=0;
	for (unsigned int w=0;w<r_grid;w++)
	{
		if_temp=i_f-w;
		if (if_temp<0)
		{
			break;
		}
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
				ids.push_back(getindexOfijf(ix_temp,iy_temp,if_temp));
			}
		}
	}
	//
	//
	for (unsigned int w=1;w<r_grid;w++)
	{
		if_temp=i_f+w;
		if (if_temp>floor(top_z/_cell_size))
		{
			break;
		}
	    for (unsigned int k=1;k<r_grid;k++ )
	    {
	   	  ix_temp=ix+k;
	   	  if (ix_temp>floor(top_x/_cell_size))
	   	   {
	   	  	break;
	   	   }
	   	  for (unsigned int t=0;t<r_grid;t++ )
	   	  {
	   	  	iy_temp=iy+t;
	   	  	if (iy_temp>floor(top_y/_cell_size))
	   	  	{
	   	  		break;
	   	  	}
	   	  	ids.push_back(getindexOfijf(ix_temp,iy_temp,if_temp));
	   	  }
	   }
	}
	return ids;
}
//////////////////////////////////////////////////////////////////////////
////输入xy 获得所在格网，以及该点周围，半径为r的所有格网点 数组索引
//*** 注意 这里不是严格的圆半径，二是格网半径 圆半径建议采用ANN 
template <typename T, typename PointT> vector<unsigned int>   
CVoxelManagement<typename T, typename PointT>:: GetCellOfXYZ(T X,T Y,T Z,T r_grid)
{
	T leftupPointX=ptIN.x-r_grid;
	T leftupPointY=ptIN.y-r_grid;
	T leftupPointZ=ptIN.z-r_grid;

	T rightPointX =ptIN.x+r_grid;
	T rightPointY =ptIN.y+r_grid;
	T rightPointZ =ptIN.z+r_grid;

	unsigned int imax,jmax,tmax,imin,jmin,tmin;
	//得到 区间范围  
	getijfromxyz(leftupPointX,leftupPointY, leftupPointZ, imin, jmin,tmin);
	getijfromxyz(rightPointX,  rightPointY, rightPointZ, imax, jmax,tmax);

	vector<unsigned int>  tempSave;
	//依据获得的区域  索引分割的区域
	for (unsigned int t=tmin;t<=tmax;t++)
	{
		for (unsigned int i=imin;i<=imax;i++)
		{
			for (unsigned int j=jmin;j<=jmax;j++)
			{
					for (unsigned  t=0;t<Square3D[t*globleColum*globleRow+i*globleColum+j].IdArray.size();t++)
					{
						tempSave.push_back(Square3D[t*globleColum*globleRow+i*globleColum+j].IdArray[t]);
					}
			}
		}
	}
	return tempSave;
}