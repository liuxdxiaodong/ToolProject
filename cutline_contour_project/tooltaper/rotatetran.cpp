#include "rotatetran.h"

using namespace std;
using namespace cv;

void rotateTrans(
	double xAxisSpeed,
	double aAxisSpeed,
	double time,
	Mat& transMat,
	Mat& rotateMat
	)
{
	double aAxisAngle = aAxisSpeed * time;
	double xAxisLength = xAxisSpeed * time;
	
	Mat rotateAxis = (Mat_<double>(3,1)<< aAxisAngle,0,0);
	Mat rotateMat;
	Mat transMat = (Mat_<double>(3,1)<< xAxisLength,0,0);
	Rodrigues(rotateAxis,rotateMat);
}