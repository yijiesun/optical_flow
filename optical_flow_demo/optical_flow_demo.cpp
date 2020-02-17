// optical_flow_demo.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "include.h"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <sstream>
#include <iomanip>
using namespace std;
using namespace cv;

//֡�������
//class FrameProcessor {
//public:
//	virtual void process(Mat &input, Mat &ouput) = 0;
//};

//���������࣬�̳���֡�������
class FeatureTracker : public FrameProcessor {
	Mat gray;  //��ǰ�Ҷ�ͼ
	Mat gray_prev;  //֮ǰ�ĻҶ�ͼ
	vector<Point2f> points[2];//ǰ����֡��������
	vector<Point2f> initial;//��ʼ������
	vector<Point2f> features;//��⵽������
	int max_count; //Ҫ���������������Ŀ
	double qlevel; //��������ָ��
	double minDist;//������֮����С���̾���
	vector<uchar> status; //��������״̬
	vector<float> err; //����ʱ�Ĵ���
public:
	FeatureTracker() :max_count(500), qlevel(0.01), minDist(10.) {}
	void process(Mat &frame, Mat &output) {
		//�õ��Ҷ�ͼ
		cvtColor(frame, gray, CV_BGR2GRAY);
		frame.copyTo(output);
		//������̫���ˣ����¼��������
		if (addNewPoint()) {
			detectFeaturePoint();
			//�����⵽��������
			//if(points[0].size()<features.size())
			//points[0].resize(points[0].size()+features.size());
			points[0].insert(points[0].end(), features.begin(), features.end());
			//initial.resize(initial.size() + features.size());
			initial.insert(initial.end(), features.begin(), features.end());
		}
		//��һ֡
		if (gray_prev.empty()) {
			gray.copyTo(gray_prev);
		}
		//����ǰ����֡�Ҷ�ͼ����ǰһ֡�������ڵ�ǰ֡��λ��
		//Ĭ�ϴ�����15*15
		calcOpticalFlowPyrLK(
			gray_prev,//ǰһ֡�Ҷ�ͼ
			gray,//��ǰ֡�Ҷ�ͼ
			points[0],//ǰһ֡������λ��
			points[1],//��ǰ֡������λ��
			status,//�����㱻�ɹ����ٵı�־
			err);//ǰһ֡�������С����͵�ǰ������С�����Ĳ���ݲ�Ĵ�С��ɾ����Щ�˶��仯���ҵĵ�
		int k = 0;
		//ȥ����Щδ�ƶ���������
		for (int i = 0; i<points[1].size(); i++) {
			if (acceptTrackedPoint(i)) {
				initial[k] = initial[i];
				points[1][k++] = points[1][i];
			}
		}
		points[1].resize(k);
		initial.resize(k);
		//��Ǳ����ٵ�������
		handleTrackedPoint(frame, output);
		//Ϊ��һ֡���ٳ�ʼ�������㼯�ͻҶ�ͼ��
		std::swap(points[1], points[0]);
		cv::swap(gray_prev, gray);
	}

	void detectFeaturePoint() {
		goodFeaturesToTrack(gray,//ͼƬ
			features,//���������
			max_count,//�����������Ŀ
			qlevel,//����ָ��
			minDist);//��С���̾���
	}
	bool addNewPoint() {
		//����������Ŀ����10����������������
		return points[0].size() <= 10;
	}

	//����������ǰ����֡�ƶ��ˣ�����Ϊ�õ���Ŀ��㣬�ҿɱ�����
	bool acceptTrackedPoint(int i) {
		return status[i] &&
			(abs(points[0][i].x - points[1][i].x) +
				abs(points[0][i].y - points[1][i].y) >2);
	}

	//��������
	void  handleTrackedPoint(Mat &frame, Mat &output) {
		for (int i = 0; i<points[1].size(); i++) {
			//��ǰ�����㵽��ʼλ����ֱ�߱�ʾ
			line(output, initial[i], points[1][i], Scalar::all(0));
			//��ǰλ����Ȧ���
			circle(output, points[1][i], 3, Scalar::all(0), (-1));
		}
	}
};


//class VideoProcessor {
//private:
//	VideoCapture caputure;
//	//д��Ƶ������
//	VideoWriter writer;
//	//����ļ���
//	string Outputfile;
//
//	int currentIndex;
//	int digits;
//	string extension;
//	FrameProcessor *frameprocessor;
//	//ͼ������ָ��
//	void(*process)(Mat &, Mat &);
//	bool callIt;
//	string WindowNameInput;
//	string WindowNameOutput;
//	//��ʱ
//	int delay;
//	long fnumber;
//	//��frameToStopֹͣ
//	long frameToStop;
//	//��ͣ��־
//	bool stop;
//	//ͼ��������Ϊ������Ƶ��
//	vector<string> images;
//	//������
//public:
//	VideoProcessor() : callIt(true), delay(0), fnumber(0), stop(false), digits(0), frameToStop(-1) {}
//	//����ͼ������
//	void setFrameProcessor(void(*process)(Mat &, Mat &)) {
//		frameprocessor = 0;
//		this->process = process;
//		CallProcess();
//	}
//	//����Ƶ
//	bool setInput(string filename) {
//		fnumber = 0;
//		//���Ѵ򿪣��ͷ����´�
//		caputure.release();
//		return caputure.open(filename);
//	}
//	//����������Ƶ���Ŵ���
//	void displayInput(string wn) {
//		WindowNameInput = wn;
//		namedWindow(WindowNameInput);
//	}
//	//���������Ƶ���Ŵ���
//	void displayOutput(string wn) {
//		WindowNameOutput = wn;
//		namedWindow(WindowNameOutput);
//	}
//	//���ٴ���
//	void dontDisplay() {
//		destroyWindow(WindowNameInput);
//		destroyWindow(WindowNameOutput);
//		WindowNameInput.clear();
//		WindowNameOutput.clear();
//	}
//
//	//����
//	void run() {
//		Mat frame;
//		Mat output;
//		if (!isOpened())
//			return;
//		stop = false;
//		while (!isStopped()) {
//			//��ȡ��һ֡
//			if (!readNextFrame(frame))
//				break;
//			if (WindowNameInput.length() != 0)
//				imshow(WindowNameInput, frame);
//			//�����֡
//			if (callIt) {
//				if (process)
//					process(frame, output);
//				else if (frameprocessor)
//					frameprocessor->process(frame, output);
//			}
//			else {
//				output = frame;
//			}
//			if (Outputfile.length()) {
//				cvtColor(output, output, CV_GRAY2BGR);
//				writeNextFrame(output);
//			}
//			if (WindowNameOutput.length() != 0)
//				imshow(WindowNameOutput, output);
//			//������ͣ��������������
//			if (delay >= 0 && waitKey(delay) >= 0)
//				waitKey(0);
//			//����ָ����ͣ�����˳�
//			if (frameToStop >= 0 && getFrameNumber() == frameToStop)
//				stopIt();
//		}
//	}
//	//��ͣ����λ
//	void stopIt() {
//		stop = true;
//	}
//	//��ѯ��ͣ��־λ
//	bool isStopped() {
//		return stop;
//	}
//	//������Ƶ�򿪱�־
//	bool isOpened() {
//		return  caputure.isOpened() || !images.empty();
//	}
//	//������ʱ
//	void setDelay(int d) {
//		delay = d;
//	}
//	//��ȡ��һ֡
//	bool readNextFrame(Mat &frame) {
//		if (images.size() == 0)
//			return caputure.read(frame);
//		else {
//			if (itImg != images.end()) {
//				frame = imread(*itImg);
//				itImg++;
//				return frame.data ? 1 : 0;
//			}
//			else
//				return false;
//		}
//	}
//
//	void CallProcess() {
//		callIt = true;
//	}
//	void  dontCallProcess() {
//		callIt = false;
//	}
//	//����ֹͣ֡
//	void stopAtFrameNo(long frame) {
//		frameToStop = frame;
//	}
//	// ��õ�ǰ֡��λ��
//	long getFrameNumber() {
//		long fnumber = static_cast<long>(caputure.get((CV_CAP_PROP_POS_FRAMES)));
//		return fnumber;
//	}
//
//	//���֡��С
//	Size getFrameSize() {
//		if (images.size() == 0) {
//			// ����Ƶ�����֡��С
//			int w = static_cast<int>(caputure.get(CV_CAP_PROP_FRAME_WIDTH));
//			int h = static_cast<int>(caputure.get(CV_CAP_PROP_FRAME_HEIGHT));
//			return Size(w, h);
//		}
//		else {
//			//��ͼ����֡��С
//			cv::Mat tmp = cv::imread(images[0]);
//			return (tmp.data) ? (tmp.size()) : (Size(0, 0));
//		}
//	}
//
//	//��ȡ֡��
//	double getFrameRate() {
//		return caputure.get(CV_CAP_PROP_FPS);
//	}
//	vector<string>::const_iterator itImg;
//	bool setInput(const vector<string> &imgs) {
//		fnumber = 0;
//		caputure.release();
//		images = imgs;
//		itImg = images.begin();
//		return true;
//	}
//
//	void  setFrameProcessor(FrameProcessor *frameprocessor) {
//		process = 0;
//		this->frameprocessor = frameprocessor;
//		CallProcess();
//	}
//
//	//��ñ�������
//	int getCodec(char codec[4]) {
//		if (images.size() != 0)
//			return -1;
//		union { // ���ݽṹ4-char
//			int value;
//			char code[4];
//		} returned;
//		//��ñ���ֵ
//		returned.value = static_cast<int>(
//			caputure.get(CV_CAP_PROP_FOURCC));
//		// get the 4 characters
//		codec[0] = returned.code[0];
//		codec[1] = returned.code[1];
//		codec[2] = returned.code[2];
//		codec[3] = returned.code[3];
//		return returned.value;
//	}
//
//
//	bool setOutput(const string &filename, int codec = 0, double framerate = 0.0, bool isColor = true) {
//		//�����ļ���
//		Outputfile = filename;
//		//�����չ��
//		extension.clear();
//		//����֡��
//		if (framerate == 0.0) {
//			framerate = getFrameRate();
//		}
//		//��ȡ����ԭ��Ƶ�ı��뷽ʽ
//		char c[4];
//		if (codec == 0) {
//			codec = getCodec(c);
//		}
//		return writer.open(Outputfile,
//			codec,
//			framerate,
//			getFrameSize(),
//			isColor);
//	}
//
//	//�����Ƶ֡��ͼƬfileme+currentIndex.ext,��filename001.jpg
//	bool setOutput(const string &filename,//·��
//		const string &ext,//��չ��
//		int numberOfDigits = 3,//����λ��
//		int startIndex = 0) {//��ʼ����
//		if (numberOfDigits<0)
//			return false;
//		Outputfile = filename;
//		extension = ext;
//		digits = numberOfDigits;
//		currentIndex = startIndex;
//		return true;
//	}
//
//	//д��һ֡
//	void writeNextFrame(Mat &frame) {
//		//�����չ����Ϊ�գ�д��ͼƬ�ļ���
//		if (extension.length()) {
//			stringstream ss;
//			ss << Outputfile << setfill('0') << setw(digits) << currentIndex++ << extension;
//			imwrite(ss.str(), frame);
//		}
//		//��֮��д����Ƶ�ļ���
//		else {
//			writer.write(frame);
//		}
//	}
//
//};

//֡��������canny��Ե���
void canny(cv::Mat& img, cv::Mat& out) {
	//�Ҷȱ任
	if (img.channels() == 3)
		cvtColor(img, out, CV_BGR2GRAY);
	// canny�������Ե
	Canny(out, out, 100, 200);
	//��ɫ��ת�������������Щ
	threshold(out, out, 128, 255, cv::THRESH_BINARY_INV);
}


void of_demo()
{
	VideoProcessor processor;
	FeatureTracker tracker;
	//��������Ƶ
	processor.setInput("F:\\video_dataset\\test0.avi");
	processor.displayInput("Current Frame");
	processor.displayOutput("Output Frame");
	//����ÿһ֡����ʱ
	processor.setDelay(1000. / processor.getFrameRate());
	//����֡����������������
	processor.setFrameProcessor(&tracker);
	//   processor.setOutput ("./bikeout.avi");
	//    processor.setOutput ("bikeout",".jpg");
	processor.run();
}

