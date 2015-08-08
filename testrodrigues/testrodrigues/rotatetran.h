#include <iostream>
#include <cstring>
#include "common.h"

/*对理想螺旋线做投影*/

struct Parameter
{
	double cutterLen;		//棒料长度
	double radius;        //铣刀半径
	double cutEdgelen;		//切削刃长度
	double taperAngel;		//锥度角度
	double helixAngel;		//螺旋角
	double pointSteplen;	//精度(每隔多少mm取一个点)
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