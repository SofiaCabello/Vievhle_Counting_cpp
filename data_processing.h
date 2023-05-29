#pragma once
#include "geometry.h"
#include <vector>
#include <string>
using namespace std;

class trail {		
public:
	int start_frame;		//起始帧
	int end_frame;			//结束帧
	int direction;			//方向
	vector<int>cover_polygon;	
	vector<int>in_polygon;		
	trail(int);		//含参构造函数
	vector<rect> locations;		//汽车轨迹（矩形形式）数组
	void insert(rect);			//两种插入函数
	void insert(vector<rect>);
	void get_direction();		//汽车方向函数
};

void readfile(vector<polygon>&, vector<vector<rect>>&, string);

vector<trail>get_trail(vector<vector<rect>>&, vector<polygon>&);

void cnt(vector<trail>&, int, int, string);