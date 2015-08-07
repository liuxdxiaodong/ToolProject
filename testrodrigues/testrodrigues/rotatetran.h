#include <iostream>
#include <cstring>
#include "common.h"

/*对理想螺旋线做投影*/

struct Parameter
{
	double cutterLen;		//棒料长度
	double diameter;        //铣刀直径
	double cutEdgelen;		//切削刃长度
	double taperAngel;		//锥度角度
	double helixAngel;		//螺旋角
	double pointSteplen;	//精度(每隔多少mm取一个点)
	double measurePos;
};

struct Offset
{
	double xAxisOffset;
	double yAxisOffset;
	double zAxisOffset;
	double aAxisOffset;
	int widthOffset;
	int heightOffset;
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

//void getPartialPoints(
//	Parameter cutterPara,
//	cv::Mat cutVecMat,
//	double xAxis,
//	double width,
//	cv::Mat partialPoints
//	);

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

//void showLocalImage(
//	cv::vector<cv::vector<double>> iamgePoints
//	);

void showImage(
	double xAxis,
	Parameter cutterPara,
	cv::vector<cv::vector<double>> imagePoints
	);

void readVideo();

void testBladeVideo(
	double rotationSpeed,
	double visualWidth,
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

void testEdgeVideo(
	double visualWidth,
	int indexCenter,
	Parameter cutterPara,
	cv::Mat inTransMat,
	cv::Mat outTransMat,
	cv::Mat RotateMat
	);

void testPicture(
	Parameter cutterPara,
	cv::Mat inTransMat,
	cv::Mat outTransMat,
	cv::Mat RotateMat,
	cv::Mat cutVecMat
	);

void testPro();

void on_mouse(
	int event,
	int x,
	int y,
	int flags,
	void* param
	);