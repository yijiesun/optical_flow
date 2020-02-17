// optical_flow_demo.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "include.h"
#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;

#define MAX_CORNERS 1000

using namespace std;
using namespace cv;

//��ƵopticalFlowFarnebackProcess�࣬�̳���֡�������
class OF_Farneback : public FrameProcessor {
	Mat gray;  //��ǰ�Ҷ�ͼ
	Mat gray_prev;  //֮ǰ�ĻҶ�ͼ

public:
	OF_Farneback(){}
	void process(Mat &frame, Mat &output) {
		//�õ��Ҷ�ͼ
		cvtColor(frame, gray, CV_BGR2GRAY);
		frame.copyTo(output);
		//������̫���ˣ����¼��������
		//��һ֡
		if (gray_prev.empty()) {
			gray.copyTo(gray_prev);
		}
		//���ܹ���
		Mat flow;
		calcOpticalFlowFarneback(gray_prev, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
		cout << flow.size() << endl;  //��ԭͼ��ÿ�����ض��������
		for (size_t y = 0; y<gray_prev.rows; y += 10) {
			for (size_t x = 0; x<gray_prev.cols; x += 10) {
				Point2f fxy = flow.at<Point2f>(y, x);
				line(output, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), CV_RGB(0, 255, 0), 1, 8);
			}
		}

		//imshow("���ܹ�����", output);
		//imwrite("E:\\SIMIT\\2020_new_project\\zsxl\\dataset\\result\\���ܹ���.jpg", output);
		//waitKey(100000);

		cv::swap(gray_prev, gray);
	}

};

class OF_PyrLK : public FrameProcessor {
	Mat gray;  //��ǰ�Ҷ�ͼ
	Mat gray_prev;  //֮ǰ�ĻҶ�ͼ
	vector<Point2f> point[2];
	vector<uchar> status;
	vector<float> err;
public:
	OF_PyrLK() {}
	~OF_PyrLK() {
		point[0].swap(vector<Point2f>());
		point[1].swap(vector<Point2f>());
		status.swap(vector<uchar>());
		err.swap(vector<float>());
	}
	void process(Mat &frame, Mat &output) {
		//�õ��Ҷ�ͼ
		cvtColor(frame, gray, CV_BGR2GRAY);
		frame.copyTo(output);
		//��һ֡
		if (gray_prev.empty()) {
			gray.copyTo(gray_prev);
		}
		//��Ǵ����������㲢��ʾ
		
		double qualityLevel = 0.01;
		double minDistance = 10;
		//��imgs[0]�еļ�⵽�Ľǵ����point[0]��
		goodFeaturesToTrack(gray_prev, point[0], MAX_CORNERS, qualityLevel, minDistance);
		cout << point[0].size() << endl;

		//ϡ�����
		TermCriteria criteria = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);


		calcOpticalFlowPyrLK(gray_prev, gray, point[0], point[1], status, err, Size(15, 15), 3, criteria);

		for (size_t i = 0; i<point[0].size() && i<point[1].size(); i++) {
			line(output, Point(cvRound(point[0][i].x), cvRound(point[0][i].y)), Point(cvRound(point[1][i].x),
				cvRound(point[1][i].y)), cvScalar(0, 50, 200), 1, CV_AA);
		}

		//imshow("ϡ�������", output);
		//imwrite("E:\\SIMIT\\2020_new_project\\zsxl\\dataset\\result\\ϡ�����.jpg", output);
		//waitKey(100000);

		cv::swap(gray_prev, gray);
		point[0].clear();
		point[1].clear();
		status.clear();
		err.clear();
	}

};
void pic_opticalFlowFarnebackProcess()
{
	//��ȡ����ͼƬ
	vector<Mat> imgs, grayImgs;
	Mat img = imread("E:\\SIMIT\\2020_new_project\\zsxl\\dataset\\1.jpg");
	imgs.push_back(img);
	img = imread("E:\\SIMIT\\2020_new_project\\zsxl\\dataset\\2.jpg");
	imgs.push_back(img);

	//�ҶȻ�
	for (size_t i = 0; i<imgs.size(); i++) {
		//����ԭ����ͼƬ
		Mat temp;
		temp.create(imgs[i].rows, imgs[i].cols, CV_8UC1);

		cvtColor(imgs[i], temp, CV_RGB2GRAY);
		grayImgs.push_back(temp);
	}

	//���ܹ���
	Mat flow;
	calcOpticalFlowFarneback(grayImgs[0], grayImgs[1], flow, 0.5, 3, 15, 3, 5, 1.2, 0);
	cout << flow.size() << endl;  //��ԭͼ��ÿ�����ض��������

	for (size_t y = 0; y<imgs[0].rows; y += 10) {
		for (size_t x = 0; x<imgs[0].cols; x += 10) {
			Point2f fxy = flow.at<Point2f>(y, x);
			line(imgs[0], Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), CV_RGB(0, 255, 0), 1, 8);
		}
	}

	imshow("���ܹ�����", imgs[0]);
	imwrite("E:\\SIMIT\\2020_new_project\\zsxl\\dataset\\result\\���ܹ���.jpg", imgs[0]);

	waitKey(100000);
}

void pic_opticalFlowPyrLKProcess()
{
	//��ȡ����ͼƬ
	vector<Mat> imgs, grayImgs;
	Mat img = imread("E:\\SIMIT\\2020_new_project\\zsxl\\dataset\\1.jpg");
	imgs.push_back(img);
	img = imread("E:\\SIMIT\\2020_new_project\\zsxl\\dataset\\2.jpg");
	imgs.push_back(img);

	//�ҶȻ�
	for (size_t i = 0; i<imgs.size(); i++) {
		//����ԭ����ͼƬ
		Mat temp;
		temp.create(imgs[i].rows, imgs[i].cols, CV_8UC1);

		cvtColor(imgs[i], temp, CV_RGB2GRAY);
		grayImgs.push_back(temp);
	}
	//�����Ƿ���ת��Ϊ�Ҷ�ͼ����Ϊopencv�����������ǻ��ڻҶ�ͼ�ģ�
	for (size_t i = 0; i<imgs.size() && i<grayImgs.size(); i++) {
		//imshow("origin",imgs[i]);
		//imshow("gray",grayImgs[i]);
		//waitKey(10000);
	}

	//��Ǵ����������㲢��ʾ
	vector<Point2f> point[2];
	double qualityLevel = 0.01;
	double minDistance = 10;
	/*
	void goodFeaturesToTrack( InputArray image, OutputArray corners,
	int maxCorners, double qualityLevel, double minDistance,
	InputArray mask=noArray(), int blockSize=3,
	bool useHarrisDetector=false, double k=0.04 )
	*/
	//��imgs[0]�еļ�⵽�Ľǵ����point[0]��
	goodFeaturesToTrack(grayImgs[0], point[0], MAX_CORNERS, qualityLevel, minDistance);
	cout << point[0].size() << endl;
	/*
	void circle(CV_IN_OUT Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0);
	*/
	//��ʾ�ǵ�
	//for(size_t i= 0;i<point[0].size();i++){
	// circle(imgs[0], cvPoint(cvRound(point[0][i].x),cvRound(point[0][i].y)), 3, cvScalar(255, 0, 0), 1, CV_AA, 0);
	//}
	//imshow("detected corner", imgs[0]);
	/*
	void cv::calcOpticalFlowFarneback( InputArray _prev0, InputArray _next0,
	OutputArray _flow0, double pyr_scale, int levels, int winsize,
	int iterations, int poly_n, double poly_sigma, int flags )
	void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
	*/

	//ϡ�����
	TermCriteria criteria = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	vector<uchar> status;
	vector<float> err;

	calcOpticalFlowPyrLK(grayImgs[0], grayImgs[1], point[0], point[1], status, err, Size(15, 15), 3, criteria);

	for (size_t i = 0; i<point[0].size() && i<point[1].size(); i++) {
		line(imgs[1], Point(cvRound(point[0][i].x), cvRound(point[0][i].y)), Point(cvRound(point[1][i].x),
			cvRound(point[1][i].y)), cvScalar(0, 50, 200), 1, CV_AA);
	}
	imshow("ϡ�������", imgs[1]);
	imwrite("E:\\SIMIT\\2020_new_project\\zsxl\\dataset\\result\\ϡ�����.jpg", imgs[1]);
	waitKey(100000);
}

void vid_opticalFlowFarnebackProcess()
{
	VideoProcessor processor;
	OF_Farneback of_fb;
	//��������Ƶ
	processor.setInput("F:\\video_dataset\\test1.avi");
	processor.displayInput("Current Frame");
	processor.displayOutput("Output Frame");
	//����ÿһ֡����ʱ
	processor.setDelay(1000. / processor.getFrameRate());
	//����֡����������������
	processor.setFrameProcessor(&of_fb);
	processor.run_syj();
}

void vid_opticalFlowPyrLKProcess()
{
	VideoProcessor processor;
	OF_PyrLK of_plk;
	//��������Ƶ
	processor.setInput("F:\\video_dataset\\test0.avi");
	processor.displayInput("Current Frame");
	processor.displayOutput("Output Frame");
	//����ÿһ֡����ʱ
	processor.setDelay(1000. / processor.getFrameRate());
	//����֡����������������
	processor.setFrameProcessor(&of_plk);
	processor.run_syj();
}