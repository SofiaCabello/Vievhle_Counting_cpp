#pragma once
#include "geometry.h"
#include <vector>
#include <string>
using namespace std;

class trail {		
public:
	int start_frame;		//��ʼ֡
	int end_frame;			//����֡
	int direction;			//����
	vector<int>cover_polygon;	
	vector<int>in_polygon;		
	trail(int);		//���ι��캯��
	vector<rect> locations;		//�����켣��������ʽ������
	void insert(rect);			//���ֲ��뺯��
	void insert(vector<rect>);
	void get_direction();		//����������
};

void readfile(vector<polygon>&, vector<vector<rect>>&, string);

vector<trail>get_trail(vector<vector<rect>>&, vector<polygon>&);

void cnt(vector<trail>&, int, int, string);