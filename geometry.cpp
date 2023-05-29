#pragma once			
#include <vector>
#include <math.h>
#include "geometry.h"
using namespace std;
int max(int x, int y) {
	return x > y ? x : y;
}
int min(int x, int y) {
	return x < y ? x : y;
}
point::point() {		//设定默认坐标以及默认车道线状况
	x = -1;
	y = -1;
	in_polygon = -1;
}
point::point(int x_, int y_) {		
	x = x_;
	y = y_;
	in_polygon = -1;
}
point::point(int x_, int y_,int in_polygon_) {
	x = x_;
	y = y_;
	in_polygon = in_polygon_;
}
point::point(const point& p) {
	x = p.x;
	y = p.y;
	in_polygon = p.in_polygon;
}
point point::operator +(point&p){
	return point(x+p.x,y+p.y);
}
point point::operator /(int scale) {
	return point(int(round(x/scale)), int(round(y / scale)));
}
rect::rect() {}
rect::rect(int tlx, int tly, int brx, int bry) {
	top_left = point(tlx, tly);
	bottom_right = point(brx, bry);
	center = (top_left + bottom_right) / 2;
}
rect::rect(point& tl, point& br) {
	top_left = tl;
	bottom_right = br;
	center = (tl + br) / 2;
}

polygon::polygon() {}


void polygon::insert(point new_point) {		//向多边形点组中添加新的点
	points.push_back(new_point);			
}

int polygon::is_in(point&p) {						//扫描线算法~
	int left = 0, right = 0;
	int length = points.size();				
	for (int i = 0; i < length; i++) {
		int x_0 = points[i].x;
		int y_0 = points[i].y;
		int x_1 = points[(i + 1) % length].x;		//方便循环计算
		int y_1 = points[(i + 1) % length].y;
		if (max(y_0, y_1) < p.y || min(y_0, y_1) > p.y || y_0 == p.y ) {	//完全不相交， y_0 == p.y ：一条线只考虑终点，防止重复计算
			continue;			
		}
		if (y_1 == p.y) {			//终点和判定点在同一水平线上的情况
			if (p.x == x_1)			//重合！
				return 1;			
			if (p.x < x_1) {		
				right += 1;
			}
			else {
				left += 1;
			}
			continue;
		}
		if (x_0 == x_1) {			//终点和判定点在同竖线上的情况（斜率不存在
			if (x_0 < p.x) {
				left += 1;
			}
			else {
				right += 1;
			}
		}
		else {
			float k = 1.0 * (y_1 - y_0) / (x_1 - x_0);		//简单的一次函数
			float b = y_1 - k * x_1;
			double x_2 = (p.y - b) / k;
			if (x_2 < p.x) {
				left += 1;
			}
			else if (x_2 > p.x) {
				right += 1;
			}
			else
				return 1;
		}
	}
	if (left % 2 == 1 && right % 2 == 1) {
		return 1;
	}
	return 0;
}
bool polygon::cover(rect&r) {					//压线
	int w = r.bottom_right.x - r.top_left.x;	//求矩形框宽width
	int h = r.bottom_right.y - r.top_left.y;	//求矩形框高height
	if (w / h < 0.4 || w / h> 2.5)				//去除不正常的宽高比
		return false;
	point judge_line_left = point(r.top_left.x + w / 5, r.bottom_right.y - h / 4);			//近似轮胎的算法
	point judge_line_right = point(r.bottom_right.x - w / 5, r.bottom_right.y - h / 4);
	int left = 0, right = 0;
	int length = points.size();
	if (is_in(judge_line_left) + is_in(judge_line_right) != 1)	//只有一个在里面才会压线
		return false;
	for (int i = 0; i < length; i++) {			//去除压前线的情况
		int x_0 = points[i].x;
		int y_0 = points[i].y;
		int x_1 = points[(i + 1) % length].x;
		int y_1 = points[(i + 1) % length].y;
		if (abs(y_1-y_0)>25) {					//去除y差值大的，留下前线
			continue;
		}
		float k = 1.0 * (y_1 - y_0) / (x_1 - x_0);
		float b = y_1 - k * x_1;
		double x_2 = (r.bottom_right.y - h / 4 - b) / k;
		if (x_2 > judge_line_left.x && x_2 < judge_line_right.x)	//判断是否压前线
			return false;
	}
	return true;
}

float iou(rect& rect1, rect& rect2) {			//交并比计算
	int area1 = (rect1.bottom_right.y - rect1.top_left.y) * (rect1.bottom_right.x - rect1.top_left.x);
	int area2 = (rect2.bottom_right.y - rect2.top_left.y) * (rect2.bottom_right.x - rect2.top_left.x);
	int sum_area = area1 + area2;
	int left = max(rect1.top_left.x, rect2.top_left.x);
	int right = min(rect1.bottom_right.x, rect2.bottom_right.x);
	int top = max(rect1.top_left.y, rect2.top_left.y);
	int bottom = min(rect1.bottom_right.y, rect2.bottom_right.y);
	if (left >= right || top >= bottom)
		return 0;
	int intersect = (right - left) * (bottom - top);
	return 1.0 * intersect / (sum_area - intersect);
}
vector<rect> linear_interpolation(rect& start, rect& end, int num) {		//补充丢失帧中的矩形
	vector<rect> inter;
	int dtl_x = end.top_left.x - start.top_left.x;
	int dtl_y = end.top_left.y - start.top_left.y;
	int dbr_x = end.bottom_right.x - start.bottom_right.x;
	int dbr_y = end.bottom_right.y - start.bottom_right.y;
	for (int i = 0; i < num; i++) {
		float scale = 1.0 * (i + 1) / (num + 1);
		int new_tl_x = start.top_left.x + dtl_x * scale;
		int new_tl_y = start.top_left.y + dtl_y * scale;
		int new_br_x = start.bottom_right.x + dbr_x * scale;
		int new_br_y = start.bottom_right.y + dbr_y * scale;
		inter.push_back(rect(new_tl_x, new_tl_y, new_br_x, new_br_y));
	}
	return inter;
}

	 