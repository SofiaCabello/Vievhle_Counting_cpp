#include "geometry.h"
#include "data_processing.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
using namespace std;
trail::trail(int start) {		//��ʼ���켣��Ϣ	
	start_frame = start;
	end_frame = start;
	direction = -1;
}
void trail::insert(rect new_rect) {		//�����µ�֡������)
	locations.push_back(new_rect);
	end_frame += 1;			//����֡��+1
}
void trail::insert(vector<rect> new_rects) {	//����һ���µ�֡�����Σ�
	int length = new_rects.size();
	for (int i = 0; i < length; i++) {
		locations.push_back(new_rects[i]);
	}
	end_frame += length;
}
void trail::get_direction() {	//Ѱ�ҹ켣�����е��������յ��γɵ��ߵļнǵ����ֵ
	vector<float>length_sum;	
	length_sum.push_back(0);
	int dx = locations[0].center.x - locations[locations.size() - 1].center.x;
	int dy = locations[0].center.y - locations[locations.size() - 1].center.y;
	float distance = sqrtf(dx * dx + dy * dy);		//�������-�յ����
	if (distance < 40) {							//���Զ̵Ĺ켣
		direction = 3;		//1��2��3����0ֱ��
		return;
	}
	float max_angle = 0;	//��ǰ����Ϊ0
	float pi = 3.14159;
	int max_index = -1;		//���Ƕ�Ӧ����±�
	int move_index = 0;		//��ʼ�ƶ�ʱ��֡��
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
		if (distance_2  < distance / 4) {		//����������ʼ�������������
			move_index = i;		//������ʼ֡��
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
		if (fabs(theta) > fabs(max_angle)) {		//���½ǵ����ֵ�Լ���Ӧ�±�
			max_index = i;
			max_angle = theta;
		}
	}
	if (max_angle > pi / 18)		//��ת
		direction = 1;
	else if (max_angle < -pi / 18)	//��ת
		direction = 2;
	else
		direction = 0;
}


vector<string> split(string str) {		//�ַ����ָ�������ڶ�ȡ�ļ�
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

//ʹ����Ƕ�׵�vector�����Ϊÿ֡�����ݣ��ڲ�Ϊһ֡�ڵľ������顣
//���������Ϊ�˷��ع켣�����а����������Ŀ����ĵ�֡��ƥ���㷨
vector<trail> get_trail(vector<vector<rect>>& frames, vector<polygon>& polygons) {		
	int delay = 10;
	vector<trail> trails;			//�켣�б�
	vector<vector<int>> scanfed;	//����Ѿ�ɨ�����λ�ã�ɨ���Ϊ1��δɨ��Ϊ0
	int frame_cnt = frames.size();	//��֡��
	int in_cnt = 0;

	for (int i = 0; i < frame_cnt; i++) {	//��ʼ������scanfedΪ0
		vector<int> tmp;
		for (int j = 0; j < frames[i].size(); j++) {
			tmp.push_back(0);
		}
		scanfed.push_back(tmp);
	}

	for (int i = 0; i < frames[0].size(); i++) {	//��ʼ����һ֡�ڵ����о���
		trail tmp(0);		
		tmp.insert(rect(frames[0][i]));
		trails.push_back(tmp);
	}

	for (int frame = 1; frame < frame_cnt; frame++) {	//ѭ������ÿһ֡
		for (int trail_index = 0; trail_index < trails.size(); trail_index++) {		//ѭ������ÿ���켣
			if (trails[trail_index].end_frame > frame || trails[trail_index].end_frame + delay + 1 < frame) {	//�켣�Ѿ������˸�֡���켣����Ѿ���delay����
				continue;
			}
			int data_index = 0;
			for (; data_index < frames[frame].size(); data_index++) {		//��֡��ÿ�������켣��������
				if (scanfed[frame][data_index] == 1) {		//��ɨ�裬����
					continue;
				}
				if (iou(frames[frame][data_index], trails[trail_index].locations[trails[trail_index].locations.size() - 1]) >= 0.6) {
					scanfed[frame][data_index] = 1;		//���Ϊ��ɨ��
					trails[trail_index].insert(frames[frame][data_index]);	//����֡��������켣
					break;
				}
			}
			if (data_index == frames[frame].size()) {		//���10֡����
				int finded = 0;
				int next_frame = min(1, frame_cnt - 1 - frame);			//�������鲻Խ��
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
		for (int data_index = 0; data_index < frames[frame].size(); data_index++) {		//δƥ���ϣ������µĹ켣
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
		trails[i].get_direction();			//�켣����ȷ��
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

void readfile(vector<polygon>& p, vector<vector<rect>>& f,string path) {	//���ļ�
	ifstream data(path, ios::in);	//���ļ�
	string line;
	stringstream s;
	getline(data, line);	//��ȡ��һ��
	int polygon_cnt = std::stoi(line);	//����һ��ת��Ϊ��ֵ���������������������Σ�
	for (int i = 0; i < polygon_cnt; i++) {		//ѭ����ȡÿ�������
		int w = 1280, h = 720;		//���廭���Ĵ�С
		polygon new_polygon;	//�����µĶ����
		getline(data, line);	//��������һ��
		int point_cnt = std::stoi(line)/2;		//����=������/2
		getline(data, line);	//����һ��
		vector<string> l = split(line);		//�з��ַ������洢
		for (int j = 0; j < point_cnt; j++) {	//������������
			int x = std::stoi(l[j * 2]);
			int y = std::stoi(l[j * 2 + 1]);
			if (x < 10)		//�����������ڻ���֮��
				x = 0;
			if (x > w - 10)
				x = w;
			if (y < 10)
				y = 0;
			if (y > h - 10)
				y = h;
			new_polygon.insert(point(x,y));		//��ӵ�
		}
		p.push_back(new_polygon);		//�Ѷ������ӵ�Ŀ��vector��
	}

	getline(data, line);	//����һ��
	int frame_cnt = std::stoi(line);		//��֡��
	for (int i = 0; i < frame_cnt; i++) {
		vector<rect> new_cars;
		getline(data, line);
		int car_cnt =std::stoi(line);	//֡�ڳ���
		for (int j = 0; j < car_cnt; j++) {
			getline(data, line);
			vector<string> sp = split(line);
			rect tmp = rect(std::stoi(sp[0]), std::stoi(sp[1]), std::stoi(sp[2]), std::stoi(sp[3]));
			new_cars.push_back(tmp);
		}
		f.push_back(new_cars);
	}
}

void cnt(vector<trail>& trails,int polygon_num,int frame_num,string out_path) {		//�ۺϼ�������
	//���峵�����ڹ켣����in_direction,ÿ֡��������car_cnt_frame
	//��������켣����out_direction,�������ڼ���in_cover,�����������out_cover
	//ѭ����ʼ��
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
				in_direction[trails[i].in_polygon[j]][trails[i].direction] += 1;	//�������ڷ������
		}
		else {
			out_direction[trails[i].direction] += 1;	//�������ⷽ�����
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

	//��Ļ���
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

	//�ļ����
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
