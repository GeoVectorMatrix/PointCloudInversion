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
	T    minZ;            //最低点的z值
	T    maxZ;            //最高点的Z
	T    maxzX;            //最高点的Z
	T    maxzY;            //最高点的Z
	T    minzY;           //最低点的y坐标
	T    minzX;           //最低点的x坐标
	//
	T    Recxmin;         //矩形区域的边界
	T    Recymin;
	T    Recxmax;         //矩形区域的边界
	T    Recymax;
	vector<unsigned int>      IdArray;       //一个区域的所有数据点的ID  数据最大
};

template <typename T> 
CSquare2D<T>::CSquare2D(void)
{
	minZ=1000000.0;
	maxZ=-1000000.0;
	minzX=0;
	minzY=0;
	maxzX =0;            //最高点的Z
	maxzY =0;            //最高点的Z
	//
	Recxmin=0;         //矩形区域的边界
	Recymin=0;
	Recxmax=0;         //矩形区域的边界
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
	unsigned int    globleRow;                    //行数
	unsigned int    globleColum;                  //列数目
	bool   is_divided;  //是否分割

public:
	void Do_Grid(const vector<PointT> &pt,const vector<unsigned int> &ids); //需要输入每个点特定的iD
	void Do_Grid(const vector<PointT> &pt);  //默认ID为序号
public:
	unsigned int           GetCellOfXY(T X,T Y);                      //输入，xy，获得对应格网的数组位置
	vector<unsigned int>   GetCellOfXY(T X,T Y,unsigned int r_grid);  //输入xy 获得所在格网，以及格网周围r个格网的 数组索引
	vector<unsigned int>   GetCellOfXY(T X,T Y,T r_grid);  //输入xy 获得所在格网，以及该点周围，半径为r的所有格网点 数组索引
	//
	/************************************************************************/
	/*       由 ij  得到的是数组中的索引                                     */
	/************************************************************************/
	unsigned int getindexOfij(unsigned int I, unsigned int J);
	int          getindexOfijSelf(unsigned int I,unsigned int J); // Boundary warning
	void         getIJfromLong(unsigned int &I, unsigned int &J, unsigned int longIdx);

private:
	T org_x;    T org_y; 
	T  top_x;   T top_y;
private:
	//设置边界 保证对最小空间进行分割
	void translate(const vector<PointT> &pt); 
	void updataSquare(CSquare2D<T> &needUpdata,PointT  ptup);
	//
	/************************************************************************/
	/* 给出坐标x y     获得所在的数据块  i  j                               */
	/************************************************************************/
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
	translate(pt);// 找边界

	//获得分割的结果  x  y方向分割数目  即  行数 列数 
	if (top_x<top_y) //保证 x对应长边 _grid_length===x  _grid_width===y;
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
		return; //如果输入的间隔比点云的范围还大了，显然不能做格网了
	}
	//获得数组的大小
	Square2D=new CSquare2D<T>[globleRow*globleColum];
	//进行数据分割
	unsigned int tmi=0;   unsigned int tmj=0;
	for (unsigned int i=0;i<pt.size();i++)
	{
		tmj=unsigned int(floor((pt[i].x-org_x)/_grid_length));
		tmi=unsigned int(floor((pt[i].y-org_y)/_grid_width));
		Square2D[tmi*globleColum+tmj].IdArray.push_back(ids[i]);
		updataSquare(Square2D[tmi*globleColum+tmj],pt[i]);
	}
	//格网边界
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
	translate(pt);// 找边界
	//获得分割的结果  x  y方向分割数目  即  行数 列数 
	if (top_x<top_y) //保证 x对应长边 _grid_length===x  _grid_width===y;
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
		return; //如果输入的间隔比点云的范围还大了，显然不能做格网了
	}
	//获得数组的大小
	Square2D=new CSquare2D<T>[globleRow*globleColum];
	//进行数据分割
	unsigned int tmi=0;   unsigned int tmj=0;
	for (unsigned int i=0;i<pt.size();i++)
	{
		tmj=unsigned int(floor((pt[i].x-org_x)/_grid_length));
		tmi=unsigned int(floor((pt[i].y-org_y)/_grid_width));
		Square2D[tmi*globleColum+tmj].IdArray.push_back(i);
		updataSquare(Square2D[tmi*globleColum+tmj],pt[i]);
	}
	//格网边界
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
	//  可以一次全部完成  没必要没输入一个点就就改变一次
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

////设置边界 保证对最小空间进行分割
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
	top_y=MaxY-MINY+0.0001;  //增加一个扰动，确保分割稳定性
}

//////////////////////////////////////////////////////////////////////////
////设置边界 保证对最小空间进行分割
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



template <typename T, typename PointT> void
CGridManagement<typename T, typename PointT>::getIJfromLong(unsigned int &I, unsigned int &J, unsigned int longIdx)
{
	if (longIdx < 0 )
	{
		longIdx = 0; // no-max than
	}
	if (longIdx > globleRow*globleColum - 1)
	{
		longIdx = globleRow*globleColum - 1; // no-max than
	}
	double rm = double(longIdx);
	I = unsigned int(floor(rm / double(globleColum)));
	//
	J = int(rm - double(I*globleColum));
}

//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> void
CGridManagement<typename T, typename PointT>:: getijfromxy(T X,T Y,unsigned int &i,unsigned int &j)
{
	i=unsigned int(floor((Y-org_y)/_grid_width));
	j=unsigned int(floor((X-org_x)/_grid_length));
	// 下面防止代码奔溃，做了强制的范围约束，事实上，自己用不会出现这些情况
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

//////////////////////////////////////////////////////////////////////////
// 为外部 访问格网 提供接口 
//////////////////////////////////////////////////////////////////////////
template <typename T, typename PointT> 	unsigned int
CGridManagement<typename T, typename PointT>:: GetCellOfXY(T X,T Y)
{
	unsigned int ix; unsigned int iy;
	getijfromxy(X,Y,ix,iy);
	return(getindexOfij(ix,iy));
}
//////////////////////////////////////////////////////////////////////////
////输入xy 获得所在格网，以及格网周围r个格网的 数组索引
//***
template <typename T, typename PointT> vector<unsigned int>   
CGridManagement<typename T, typename PointT>::GetCellOfXY(T X,T Y,unsigned int r_grid)
{
	vector<unsigned int>  ids;
	//
	unsigned int the_ID=GetCellOfXY(X,Y); //该点自身的id
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
//////////////////////////////////////////////////////////////////////////
////输入xy 获得所在格网，以及该点周围，半径为r的所有格网点 数组索引
//*** 注意 这里不是严格的圆半径，二是格网半径 圆半径建议采用ANN 
template <typename T, typename PointT> vector<unsigned int>   
CGridManagement<typename T, typename PointT>:: GetCellOfXY(T X,T Y,T r_grid)
{
	T leftupPointX=ptIN.x-r_grid;
	T leftupPointY=ptIN.y-r_grid;
	T rightPointX =ptIN.x+r_grid;
	T rightPointY =ptIN.y+r_grid;
	unsigned int imax,jmax,imin,jmin;
	//得到 区间范围  
	getijfromxy(leftupPointX,leftupPointY,  imin, jmin);
	getijfromxy(rightPointX,  rightPointY,  imax, jmax);

	vector<unsigned int>  tempSave;
	//依据获得的区域  索引分割的区域
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