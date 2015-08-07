#include <iostream>
#include <cstring>
#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2\opencv.hpp>

using namespace std;


struct Parameter
{
	double cutterLen;		//棒料长度
	double diameter;        //铣刀直径
	double cutEdgelen;		//切削刃长度
	double taperAngel;		//锥度角度
	double helixAngel;		//螺旋角
	double pointSteplen;	//精度(每隔多少mm取一个点)
};

/* get the Point sets of the cutline projecting to the image*/
void getPoint(
	Parameter cutterPara,
	double* arrar,
	cv::Mat imageVector, 
	const char* filename
	);

/* Get the Point sets of the contourline projecting to the image*/
void contourLine(
	Parameter cutterPara,
	double* arrar,
	cv::Mat imageVector,
	const char* filename
	);

/* project vector with Point3d to the 2D image*/
void projectImage(
	double* arrar,
	vector<cv::Point3d> vec,
	cv::Mat imageVector,
	const char* filename,
	int n
	);

void printFile(
	cv::Mat fileContent,
	const char* filename
	);

