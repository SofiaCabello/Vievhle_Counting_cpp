#pragma once
#include <vector>
using namespace std;	
class point {					//����
public:
	int x;
	int y;
	int in_polygon;
	point();					//�޲ι��캯��
	point(int, int);			//���ι��캯��������������ĵ�
	point(int, int, int);		//ͬ�ϣ��Լ�������in_polygon
	point(const point&);		//����
	point operator +(point&);	//���ؼӺ�
	point operator /(int x);	//���س���
};
class rect {					//������
public:
	point top_left;				//���ϵ�
	point bottom_right;			//���µ�
	point center;				//���ĵ�
	rect();						//�޲ι��캯��
	rect(int, int, int, int);	//�����깹�캯��
	rect(point&, point&);		//˫�㹹�캯��
};
class polygon {					//������࣬������
public:
	vector<point> points;		
	polygon();					//�޲ι��캯��
	void insert(point);			//�����
	int is_in(point&);			//�ж�һ�����Ƿ��ڳ�������
	bool cover(rect&);			//�ж��Ƿ�ѹ�ߣ�unstable
};
float iou(rect&, rect&);		//�����ȼ�����
vector<rect> linear_interpolation(rect&, rect&, int);	//���Բ�ֵ����
int max(int, int);
int min(int, int);
