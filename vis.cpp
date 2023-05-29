#include <sstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "geometry.h"
#include "data_processing.h"
using namespace std;
using namespace cv;

void visualize(string path, vector<polygon>& lines, vector<vector<rect>>& frames, vector<trail>& trails,int show_out) {
	//颜色定义
	Scalar edge = Scalar(255, 0, 0); //bgr
	Scalar in_edge= Scalar(0, 0, 255);
	Scalar out_edge = Scalar(0, 255, 0);
	Scalar straight = Scalar(200, 200, 200);
	Scalar left = Scalar(0, 255, 255);
	Scalar right = Scalar(255, 0, 255);
	Scalar ignored = Scalar(100, 200, 100);
	Scalar text = Scalar(255, 255, 0);
	
	//读视频
	Mat frame;
	VideoCapture video(path);
	int frame_cnt = 0;
	while (video.read(frame)) {
		//定义并初始化in_polygon
		vector<int> in_polygon;
		for (int i = 0; i < lines.size(); i++) {
			in_polygon.push_back(0);
		}
		for (int i = 0; i < lines.size(); i++) {
			for (int j = 0; j < lines[i].points.size(); j++) {
				line(frame, Point2i(lines[i].points[j].x, lines[i].points[j].y),
					Point2i(lines[i].points[(j + 1) % lines[i].points.size()].x,
						lines[i].points[(j + 1) % lines[i].points.size()].y), edge, 5);
			}
		}
		for (int i = 0; i < trails.size(); i++) {
			if (trails[i].start_frame > frame_cnt || trails[i].end_frame < frame_cnt)
				continue;
			if (!show_out) {		//是否展示车道线以外的车
				if (trails[i].in_polygon.size() == 0)
					continue;
			}
			Scalar color;
			
			if (trails[i].direction == 0)
				color = straight;
			else if (trails[i].direction == 1)
				color = right;
			else if (trails[i].direction == 2)
				color = left;
			else 
				color = ignored;

			for (int j = trails[i].start_frame; j < frame_cnt - 1; j++) {		//轨迹线绘制
				point center_now = trails[i].locations[j - trails[i].start_frame].center;
				point center_next = trails[i].locations[j + 1 - trails[i].start_frame].center;
				line(frame, Point2i(center_now.x, center_now.y),
					Point2i(center_next.x, center_next.y), color, 5);
			}
			point center_now = trails[i].locations[max(min(frame_cnt - 1 - trails[i].start_frame, trails[i].locations.size()), 0)].center;
			for (int k = 0; k < lines.size(); k++) {	//中心点绘制
				if (center_now.in_polygon>-1) {
					circle(frame, Point2i(center_now.x, center_now.y), 5, in_edge, 2);
					in_polygon[center_now.in_polygon] += 1;
					break;
				}
				if (k == lines.size() - 1) {
					circle(frame, Point2i(center_now.x, center_now.y), 5, out_edge, 2);
				}
			}
		}
		stringstream s;
		s << "count: ";
		for (int i = 0; i < lines.size(); i++) {
			s << in_polygon[i] << " ";
		}
		string a = s.str();
		putText(frame, s.str(), Point2i(40, 100), cv::FONT_ITALIC, 1, text, 3);
		putText(frame, "in:", Point2i(40, 130), cv::FONT_ITALIC, 1, text, 3);
		circle(frame, Point2i(120, 122), 5, in_edge, 2);
		putText(frame, "out:", Point2i(40, 160), cv::FONT_ITALIC, 1, text, 3);
		circle(frame, Point2i(120, 152), 5, out_edge, 2);
		putText(frame, "straight:", Point2i(40, 190), cv::FONT_ITALIC, 1, text, 3);
		line(frame, Point2i(173, 185), Point2i(198, 185), straight, 5);
		putText(frame, "left:", Point2i(40, 220), cv::FONT_ITALIC, 1, text, 3);
		line(frame, Point2i(173, 210), Point2i(198, 210), left, 5);
		putText(frame, "right:", Point2i(40, 250), cv::FONT_ITALIC, 1, text, 3);
		line(frame, Point2i(173, 243), Point2i(198, 243), right, 5);
		putText(frame, "ignored:", Point2i(40, 280), cv::FONT_ITALIC, 1, text, 3);
		line(frame, Point2i(173, 275), Point2i(198, 275), ignored, 5);
		frame_cnt += 1;
		imshow("video", frame);
		waitKey(1);
	}
}