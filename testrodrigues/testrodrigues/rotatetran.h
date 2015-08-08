#include <iostream>
#include <cstring>
#include "common.h"

/*��������������ͶӰ*/

struct Parameter
{
	double cutterLen;		//���ϳ���
	double radius;        //ϳ���뾶
	double cutEdgelen;		//�����г���
	double taperAngel;		//׶�ȽǶ�
	double helixAngel;		//������
	double pointSteplen;	//����(ÿ������mmȡһ����)
	double measurePos;
	int upDown;
};

void getPoint(
	Parameter cutterPara,
	double aAxisOffset,
	cv::Mat& cutVecmet
	);

void getPartialPoints(
	Parameter cutterPara,
	double xAxis,
	double visualWidth,
	int& indexCenter,
	cv::Mat cutVecMat,
	cv::Mat& partPointMat
	);

void rotateTrans(
	const double xAxis,
	const double aAxis,
	cv::Mat& inTransMat,
	cv::Mat& outTransMat,
	cv::Mat& RotateMat
	);

void pointsProject(
	cv::Mat srcPoints,
	cv::Mat inTransMat,
	cv::Mat outTransMat,
	cv::Mat RotateMat,
	cv::vector<cv::vector<double>>& imagePoints
	);
void getWidthHeight(
	char* fileName
	);

void testBladeVideo(
	double rotationSpeed,
	double visualWidth,
	char* fileName,
	Parameter cutterPara,
	cv::Mat inTransMat,
	cv::Mat outTransMat,
	cv::Mat RotateMat,
	cv::Mat cutVecMat
	);

void getEdgePoints(
	double visualWidth,
	int& indexCenter,
	Parameter cutterPara,
	cv::Mat& edgePoints
	);

void getEdgeCirclePoints(
	double visualWidth,
	int& indexCenter,
	Parameter cutterPara,
	cv::Mat& circlePoints
	);

void testEdgeVideo(
	double visualWidth,
	int indexCenter,
	char* fileName,
	Parameter cutterPara,
	cv::Mat inTransMat,
	cv::Mat outTransMat,
	cv::Mat RotateMat,
	cv::Mat edgePoints
	);

void testPro();

void on_mouse(
	int event,
	int x,
	int y,
	int flags,
	void* param
	);