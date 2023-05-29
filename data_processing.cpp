#include "geometry.h"
#include "data_processing.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
using namespace std;
trail::trail(int start) {		//初始化轨迹信息	
	start_frame = start;
	end_frame = start;
	direction = -1;
}
void trail::insert(rect new_rect) {		//插入新的帧（矩形)
	locations.push_back(new_rect);
	end_frame += 1;			//最终帧数+1
}
void trail::insert(vector<rect> new_rects) {	//插入一组新的帧（矩形）
	int length = new_rects.size();
	for (int i = 0; i < length; i++) {
		locations.push_back(new_rects[i]);
	}
	end_frame += length;
}
void trail::get_direction() {	//寻找轨迹上所有点与起点和终点形成的线的夹角的最大值
	vector<float>length_sum;	
	length_sum.push_back(0);
	int dx = locations[0].center.x - locations[locations.size() - 1].center.x;
	int dy = locations[0].center.y - locations[locations.size() - 1].center.y;
	float distance = sqrtf(dx * dx + dy * dy);		//计算起点-终点距离
	if (distance < 40) {							//忽略短的轨迹
		direction = 3;		//1右2左3忽略0直行
		return;
	}
	float max_angle = 0;	//当前最大角为0
	float pi = 3.14159;
	int max_index = -1;		//最大角对应点的下标
	int move_index = 0;		//开始移动时的帧数
	for (int i =1; i < locations.size(); i++) {
		int dx_0 = locations[i].center.x - locations[move_index].center.x;
		int dy_0 = locations[i].center.y - locations[move_index].center.y;
		int dx_1 = locations[locations.size() - 1].center.x  - locations[i].center.x;
		int dy_1 = locations[locations.size() - 1].center.y- locations[i].center.y;
		int dx_2 = locations[i].center.x - locations[0].center.x;
		int dy_2 = locations[i].center.y - locations[0].center.y;
		float distance_0 = sqrtf(dx_0 * dx_0 + dy_0 * dy_0);
		float distance_1 = sqrtf(dx_1 * dx_1 + dy_1 * dy_1);
		float distance_2 = sqrtf(dx_2 * dx_2 + dy_2 * dy_2);
		if (distance_2  < distance / 4) {		//若这个点和起始点距离过近则忽略
			move_index = i;		//更新起始帧数
			continue;
		}
		if (distance_0 < distance / 5 || distance_1 < distance / 5) {
			continue;
		}
		//int cos = (dy_0 * dy_1 + dx_0 * dx_1) / (distance_0 * distance_1);
		float theta = atan2(dy_1, dx_1) - atan2(dy_0, dx_0);
		if (theta > pi)
			theta -= pi * 2;
		if (theta < -pi)
			theta += pi * 2;
		//theta = theta * 180.0 / pi;
		if (fabs(theta) > fabs(max_angle)) {		//更新角的最大值以及对应下标
			max_index = i;
			max_angle = theta;
		}
	}
	if (max_angle > pi / 18)		//右转
		direction = 1;
	else if (max_angle < -pi / 18)	//左转
		direction = 2;
	else
		direction = 0;
}


vector<string> split(string str) {		//字符串分割函数，用于读取文件
	int size = str.size();
	vector<string> ans;
	int j = 0;
	for (int i = 0; i < size; i++) {
		if (str[i] == ' ') {
			ans.push_back(str.substr(j, i - j));
			j = i + 1;
		}
	}
	ans.push_back(str.substr(j, size - j));
	return ans;
}

//使用了嵌套的vector，外层为每帧的内容，内层为一帧内的矩形数组。
//这个函数是为了返回轨迹，其中包含了这个项目最核心的帧间匹配算法
vector<trail> get_trail(vector<vector<rect>>& frames, vector<polygon>& polygons) {		
	int delay = 10;
	vector<trail> trails;			//轨迹列表
	vector<vector<int>> scanfed;	//标记已经扫描过的位置，扫描过为1，未扫描为0
	int frame_cnt = frames.size();	//总帧数
	int in_cnt = 0;

	for (int i = 0; i < frame_cnt; i++) {	//初始化数组scanfed为0
		vector<int> tmp;
		for (int j = 0; j < frames[i].size(); j++) {
			tmp.push_back(0);
		}
		scanfed.push_back(tmp);
	}

	for (int i = 0; i < frames[0].size(); i++) {	//初始化第一帧内的所有矩形
		trail tmp(0);		
		tmp.insert(rect(frames[0][i]));
		trails.push_back(tmp);
	}

	for (int frame = 1; frame < frame_cnt; frame++) {	//循环处理每一帧
		for (int trail_index = 0; trail_index < trails.size(); trail_index++) {		//循环处理每个轨迹
			if (trails[trail_index].end_frame > frame || trails[trail_index].end_frame + delay + 1 < frame) {	//轨迹已经包括了该帧；轨迹差距已经在delay以上
				continue;
			}
			int data_index = 0;
			for (; data_index < frames[frame].size(); data_index++) {		//该帧的每个汽车轨迹进行适配
				if (scanfed[frame][data_index] == 1) {		//已扫描，跳过
					continue;
				}
				if (iou(frames[frame][data_index], trails[trail_index].locations[trails[trail_index].locations.size() - 1]) >= 0.6) {
					scanfed[frame][data_index] = 1;		//标记为已扫描
					trails[trail_index].insert(frames[frame][data_index]);	//将此帧插入该条轨迹
					break;
				}
			}
			if (data_index == frames[frame].size()) {		//向后10帧查找
				int finded = 0;
				int next_frame = min(1, frame_cnt - 1 - frame);			//保障数组不越界
				int last_frame = min(delay, frame_cnt - delay - frame);
				if (next_frame<=0)
					break;
				for (; next_frame < last_frame; next_frame++) {
					if (finded == 1)
						break;
					data_index = 0;
					for (; data_index < frames[frame + next_frame].size(); data_index++) {
						if (scanfed[frame + next_frame][data_index] == 1) {
							continue;
						}
						if (iou(frames[frame + next_frame][data_index], trails[trail_index].locations[trails[trail_index].locations.size() - 1]) >= 0.5) {
							scanfed[frame + next_frame][data_index] = 1;
							finded = 1;
							in_cnt += 1;
							vector<rect> new_rects = linear_interpolation(trails[trail_index].locations[trails[trail_index].locations.size() - 1],
								frames[frame + next_frame][data_index], next_frame);
							trails[trail_index].insert(new_rects);
							trails[trail_index].insert(frames[frame + next_frame][data_index]);
							break;
						}
					}
				}
			}
		}
		for (int data_index = 0; data_index < frames[frame].size(); data_index++) {		//未匹配上，建立新的轨迹
			if (scanfed[frame][data_index] == 0) {
				scanfed[frame][data_index] = 1;
				trail tmp(frame);
				tmp.insert(frames[frame][data_index]);
				trails.push_back(tmp);
			}
		}
	}

	int polygon_num = polygons.size();
	int trail_num = trails.size();

	for (int i = 0; i < trail_num; i++) {
		trails[i].get_direction();			//轨迹方向确认
	}

	for (int i = 0; i < trail_num; i++) {
		vector<int>tmp_cnts;
		vector<int>cover_cnts;
		for (int index = 0; index < polygons.size(); index++) {
			tmp_cnts.push_back(0);
			cover_cnts.push_back(0);
		}
		int trail_size = trails[i].locations.size();
		for (int j = 0; j < trail_size; j++) {
			for (int k = 0; k < polygon_num; k++) {
				if (polygons[k].is_in(trails[i].locations[j].center)) {
					if (tmp_cnts[k] == 0) {
						tmp_cnts[k] = 1;
						trails[i].in_polygon.push_back(k);
					}
					trails[i].locations[j].center.in_polygon = k;
				}
				if (polygons[k].cover(trails[i].locations[j])) {
					if (cover_cnts[k] == 0) {
						cover_cnts[k] = 1;
						trails[i].cover_polygon.push_back(k);
					}
					trails[i].locations[j].center.in_polygon = k;
				}
			}
		}

	}
	return trails;
}

void readfile(vector<polygon>& p, vector<vector<rect>>& f,string path) {	//读文件
	ifstream data(path, ios::in);	//打开文件
	string line;
	stringstream s;
	getline(data, line);	//读取第一行
	int polygon_cnt = std::stoi(line);	//将第一行转化为数值，即多边形数（车道多边形）
	for (int i = 0; i < polygon_cnt; i++) {		//循环读取每个多边形
		int w = 1280, h = 720;		//定义画布的大小
		polygon new_polygon;	//创建新的多边形
		getline(data, line);	//继续读下一行
		int point_cnt = std::stoi(line)/2;		//点数=坐标数/2
		getline(data, line);	//读下一行
		vector<string> l = split(line);		//切分字符串并存储
		for (int j = 0; j < point_cnt; j++) {	//处理坐标数据
			int x = std::stoi(l[j * 2]);
			int y = std::stoi(l[j * 2 + 1]);
			if (x < 10)		//将坐标限制在画布之内
				x = 0;
			if (x > w - 10)
				x = w;
			if (y < 10)
				y = 0;
			if (y > h - 10)
				y = h;
			new_polygon.insert(point(x,y));		//添加点
		}
		p.push_back(new_polygon);		//把多边形添加到目的vector中
	}

	getline(data, line);	//读下一行
	int frame_cnt = std::stoi(line);		//总帧数
	for (int i = 0; i < frame_cnt; i++) {
		vector<rect> new_cars;
		getline(data, line);
		int car_cnt =std::stoi(line);	//帧内车数
		for (int j = 0; j < car_cnt; j++) {
			getline(data, line);
			vector<string> sp = split(line);
			rect tmp = rect(std::stoi(sp[0]), std::stoi(sp[1]), std::stoi(sp[2]), std::stoi(sp[3]));
			new_cars.push_back(tmp);
		}
		f.push_back(new_cars);
	}
}

void cnt(vector<trail>& trails,int polygon_num,int frame_num,string out_path) {		//综合计数函数
	//定义车道线内轨迹方向in_direction,每帧车辆计数car_cnt_frame
	//车道线外轨迹方向out_direction,车道线内计数in_cover,车道线外计数out_cover
	//循环初始化
	int car_num = trails.size();
	vector<vector<int>>in_direction,car_cnt_frame;
	vector<int>out_direction, in_cover, out_cover;
	for (int i = 0; i < polygon_num; i++) {
		in_cover.push_back(0);
		out_cover.push_back(0);
	}
	for (int i = 0; i < frame_num; i++) {
		vector<int>tmp;
		for (int j = 0; j < polygon_num; j++) {
			tmp.push_back(0);
		}
		car_cnt_frame.push_back(tmp);
	}
	for (int i = 0; i < 4; i++) {
		vector<int>tmp0, tmp1;
		for (int j = 0; j < polygon_num; j++) {
			tmp0.push_back(0);
		}
		in_direction.push_back(tmp0);
		out_direction.push_back(0);
	}


	for (int i = 0; i < trails.size(); i++) {
		if (trails[i].in_polygon.size()) {
			for (int j = 0; j < trails[i].in_polygon.size(); j++)
				in_direction[trails[i].in_polygon[j]][trails[i].direction] += 1;	//车道线内方向计数
		}
		else {
			out_direction[trails[i].direction] += 1;	//车道线外方向计数
		}
		if (trails[i].cover_polygon.size()) {
			if (trails[i].in_polygon.size()) {
				for (int j = 0; j < trails[i].cover_polygon.size(); j++)
					in_cover[trails[i].cover_polygon[j]] += 1;
			}
			else {
				for (int j = 0; j < trails[i].cover_polygon.size(); j++)
					out_cover[trails[i].cover_polygon[j]] += 1;
			}
		}
		for (int j = 0; j < frame_num; j++) {
			if (trails[i].start_frame <= j && trails[i].end_frame > j&& trails[i].locations[j-trails[i].start_frame].center.in_polygon>-1) {
				for (int k = 0; k < trails[i].in_polygon.size(); k++) {
					car_cnt_frame[j][trails[i].in_polygon[k]] += 1;
				}
			}
		}
	}

	//屏幕输出
	cout << "#------------------------#" << endl;
	cout << "Number of the cars in the edges:" << endl;
	for (int i = 0; i < frame_num; i++) {
		cout << "frame " << i+1 << ":" << endl;
		for (int j = 0; j < polygon_num; j++) {
			cout << "edge " << j + 1 << ":" << car_cnt_frame[i][j] << endl;
		}
	}
	cout << "Total frame:" << frame_num << endl;
	cout << "#------------------------#" << endl;
	cout << "Number of the cars:" << car_num << endl;
	cout << "#------------------------#" <<endl;
	cout << "Number of the passed cars with different directions:"<< endl;
	for (int i = 0; i < in_direction.size(); i++) {
		int sum = 0;
		for (int j = 0; j < in_direction[i].size(); j++) {
			sum += in_direction[i][j];
		}
		cout << "edge " << i+1 << ":" << endl;
		cout << "total:" << sum << endl;
		cout << "straight:" << in_direction[i][0] << endl;
		cout << "right:" << in_direction[i][1] << endl;
		cout << "left:" << in_direction[i][2] << endl;
		cout << "ignored:" << in_direction[i][3] << endl;
		cout << "-------------------" << endl;
	}
	int sum = 0;
	for (int j = 0; j < out_direction.size(); j++) {
		sum += out_direction[j];
	}
	cout << "Out of the edges:" << endl;
	cout << "total:" << sum << endl;
	cout << "straight:" << out_direction[0] << endl;
	cout << "right:" << out_direction[1] << endl;
	cout << "left:" << out_direction[2] << endl;
	cout << "ignored:" << out_direction[3] << endl;
	cout << "#------------------------#" << endl;
	cout << "Number of the cars on the edges:" << endl;
	for (int i = 0; i < polygon_num; i++) {
		cout << "edge " << i+1 << ":" << endl;
		cout << "in:" << in_cover[i] << " out:" << out_cover[i] << endl;
	}
	cout << "#-----------end-----------#" << endl;

	//文件输出
	ofstream ofs;
	ofs.open(out_path,ios::out);
	ofs << "#------------------------#" << endl;
	ofs << "Number of the cars in the edges:" << car_num << endl;
	for (int i = 0; i < frame_num; i++) {
		ofs << "frame " << i+1 << ":" << endl;
		for (int j = 0; j < polygon_num; j++) {
			ofs << "edge " << j << ":" << car_cnt_frame[i][j] << endl;
		}
	}
	ofs << "Total frame:" << frame_num << endl;
	ofs << "#------------------------#" << endl;
	ofs << "Number of the cars:" << car_num << endl;
	ofs << "#------------------------#" << endl;
	ofs << "Number of the cars with different directions:" << endl;
	for (int i = 0; i < in_direction.size(); i++) {
		int sum = 0;
		for (int j = 0; j < in_direction[i].size(); j++) {
			sum += in_direction[i][j];
		}
		ofs << "edge " << i + 1 << ":" << endl;
		ofs << "total:" << sum << endl;
		ofs << "straight:" << in_direction[i][0] << endl;
		ofs << "right:" << in_direction[i][1] << endl;
		ofs << "left:" << in_direction[i][2] << endl;
		ofs << "ignored:" << in_direction[i][3] << endl;
		ofs << "-------------------" << endl;
	}
	ofs << "Out of the edges:" << endl;
	ofs << "total:" << sum << endl;
	ofs << "straight:" << out_direction[0] << endl;
	ofs << "right:" << out_direction[1] << endl;
	ofs << "left:" << out_direction[2] << endl;
	ofs << "ignored:" << out_direction[3] << endl;
	ofs << "#------------------------#" << endl;
	ofs << "Number of the cars on the edges:" << endl;
	for (int i = 0; i < polygon_num; i++) {
		ofs << "edge " << i+1 << ":" << endl;
		ofs << "in:" << in_cover[i] << " out:" << out_cover[i] << endl;
	}
	ofs << "#-----------end-----------#" << endl;
	ofs.close();
	return;
}
