#include<iostream>
#include <vector>
#include <math.h>
#include <string>
#include <sstream>
#include "geometry.h"
#include "data_processing.h"
#include "vis.h"
using namespace std;

int main()
{

	string data_path = "/Users/jintaixi/CLionProjects/Viechle_Counting_cpp/data1.txt";
	string out_path = "/Users/jintaixi/CLionProjects/Viechle_Counting_cpp/out.txt";
	string video_path = "/Users/jintaixi/CLionProjects/Viechle_Counting_cpp/video.MP4";

	int show_out_cars = 1;									//是否展示车道线以外的车，1是0否
	vector<polygon> polygons;								//车道线数组
	vector<vector<rect>> frames;							//帧数据数组
	

	readfile(polygons, frames, data_path);							//自定义文件读取函数

	vector<trail> trails=get_trail(frames,polygons);	//轨迹数组

	visualize(video_path, polygons, frames, trails, show_out_cars);	//可视化

	cnt(trails, polygons.size(), frames.size(),out_path);			//count，对所有需要计数的量进行计数，并输出和保存捏~

	return 0;
}
