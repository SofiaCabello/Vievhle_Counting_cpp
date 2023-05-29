#pragma once
#include <vector>
using namespace std;	
class point {					//点类
public:
	int x;
	int y;
	int in_polygon;
	point();					//无参构造函数
	point(int, int);			//带参构造函数，创建有坐标的点
	point(int, int, int);		//同上，以及包含了in_polygon
	point(const point&);		//复制
	point operator +(point&);	//重载加号
	point operator /(int x);	//重载除号
};
class rect {					//矩形类
public:
	point top_left;				//左上点
	point bottom_right;			//右下点
	point center;				//中心点
	rect();						//无参构造函数
	rect(int, int, int, int);	//四坐标构造函数
	rect(point&, point&);		//双点构造函数
};
class polygon {					//多边形类，车道线
public:
	vector<point> points;		
	polygon();					//无参构造函数
	void insert(point);			//点插入
	int is_in(point&);			//判断一辆车是否在车道线内
	bool cover(rect&);			//判断是否压线（unstable
};
float iou(rect&, rect&);		//交并比计算喵
vector<rect> linear_interpolation(rect&, rect&, int);	//线性插值函数
int max(int, int);
int min(int, int);
