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

	int show_out_cars = 1;									//�Ƿ�չʾ����������ĳ���1��0��
	vector<polygon> polygons;								//����������
	vector<vector<rect>> frames;							//֡��������
	

	readfile(polygons, frames, data_path);							//�Զ����ļ���ȡ����

	vector<trail> trails=get_trail(frames,polygons);	//�켣����

	visualize(video_path, polygons, frames, trails, show_out_cars);	//���ӻ�

	cnt(trails, polygons.size(), frames.size(),out_path);			//count����������Ҫ�����������м�����������ͱ�����~

	return 0;
}
