#include "rotatetran.h"

using namespace std;
using namespace cv;

int widthOffset;
int heightOffset;
int resizeCount;

// 生成旋转平移矩阵RotateMat和inTransMat,outTransMat
void rotateTrans(
	const double xAxis,
	const double aAxis,
	Mat& inTransMat,
	Mat& outTransMat,
	Mat& RotateMat
	)
{	
	Mat rotateAxis = (Mat_<double>(3,1)<< aAxis,0,0);
	inTransMat = (Mat_<double>(3,1)<< xAxis,0,0);
	outTransMat = (Mat_<double>(3,1)<< 0,0,110);
	Rodrigues(rotateAxis,RotateMat);
}

// 得到理想的刃线点
void getPoint(
	Parameter cutterPara,
	double aAxisOffset,
	cv::Mat& cutVecMat
	)
{

	double radius = cutterPara.radius;
	double lead = 2 * CV_PI * radius / tan(cutterPara.helixAngel * CV_PI / 180 );

	int numofPoint;
	double a = cutterPara.cutEdgelen / cutterPara.pointSteplen;
	numofPoint = floor(a);
	int n = numofPoint;

	vector<double> xx(n+1);
	vector<double> mtoolR(n+1);
	vector<double> mc(n+1);
	vector<double> zz(n+1); 
	vector<double> yy(n+1); 
	vector<cv::Point3d> cutVec(n+1);

	for(int i=0;i<n+1;i++)
	{
		xx[i] = 0-(double)i * cutterPara.pointSteplen;
		mtoolR[i] = xx[i] * tan(cutterPara.taperAngel * CV_PI / 180);
		mc[i] = -2 * CV_PI * xx[i] / lead + aAxisOffset;
		zz[i] = (radius + mtoolR[i]) * cos( mc[i] );
		yy[i] = -(radius + mtoolR[i]) * sin( mc[i] );
		cutVec[i] = cv::Point3d( xx[i], yy[i], zz[i] );
	}
	Mat cutterMet = Mat(cutVec).reshape(1).t();
	Mat cutVecmet1;
	Mat G = cutterMet.row(0).clone();
	Mat H = cutterMet.row(1).clone();
	Mat J = cutterMet.row(2).clone();

	vconcat(G, H, cutVecmet1); 
	vconcat(cutVecmet1, J, cutVecMat);
}

// 将得到的刃线轨迹的点分割出有用的部分，即能在图像上显示的部分
void getPartialPoints(
	Parameter cutterPara,
	double xAxis,
	double visualWidth,
	int& indexCenter,
	Mat cutVecMat,
	Mat& partPointMat
	)
{
	double xAxisPrev = xAxis - visualWidth * 2/3;
	double xAxisLater = xAxis + visualWidth * 2/3;
	int indexPrev, indexLater;

	if(xAxisPrev < 0)
	{
		indexPrev = 0;
		indexLater = int(xAxisLater/cutterPara.pointSteplen);
	}
	else if(xAxisLater > cutterPara.pointSteplen)
	{
		indexPrev = int(xAxisPrev/cutterPara.pointSteplen);
		indexLater = cutVecMat.cols;
	}
	else
	{
		indexPrev = int(xAxisPrev/cutterPara.pointSteplen);
		indexLater = int(xAxisLater/cutterPara.pointSteplen);
	}
	partPointMat = cutVecMat.colRange(indexPrev,indexLater);
	indexCenter = int(xAxis/cutterPara.pointSteplen - indexPrev);
}

// 由上述生成的旋转平移矩阵将刃线的世界坐标系转化到相机坐标系中
void pointsProject(
	Mat srcPoints,
	Mat inTransMat,
	Mat outTransMat,
	Mat RotateMat,
	vector<vector<double>>& imagePoints
	)
{
	int num = srcPoints.cols;
	Mat M = (Mat_<double>(3,3)<< 33636*2,0,2592/2+0.5,0,33636*2,1944/2+0.5,0,0,1);

	for (int i=0;i<num;i++)
	{
		Mat result = M * (RotateMat * srcPoints.col(i) + inTransMat + outTransMat);
		Mat x = result.row(0).clone();
		Mat y = result.row(1).clone();
		Mat z = result.row(2).clone();
		Mat points;
		vconcat(x.mul(1/z),y.mul(1/z),points);

		vector<double> col;
		points.copyTo(col);
		imagePoints.push_back(col);
	}
}

// 点击图像上的一点，该点为视频中的焦点，得到指导线的起始点或焦点中心的像素坐标
void getWidthHeight(
	char* fileName
	)
{
	VideoCapture cap(fileName);
	if(!cap.isOpened())
		cerr << "Can not open the file!" << endl;

	cout << "Click the focus point!" << endl;
	namedWindow("image",1);

	Mat frame,frame1;
	cap >> frame;

	resize(frame,frame1,Size(frame.cols/resizeCount,frame.rows/resizeCount));
	imshow("image",frame1);
	cv::setMouseCallback("image",on_mouse,0);
	waitKey(0);
}

// 跟踪刃线的指导线实验
void testBladeVideo(
	double rotationSpeed,
	double visualWidth,
	char* fileName,
	Parameter cutterPara,
	Mat inTransMat,
	Mat outTransMat,
	Mat RotateMat,
	Mat cutVecMat
	)
{
	VideoCapture cap(fileName);
	if(!cap.isOpened())
	{
		cerr << "Can not open the file!" << endl;
	}

	//char imageName[50];
	int i=0;
	int indexCenter;

	namedWindow("blade",1);

	for(;;)
	{
		Mat frame,frame1,framehead;
		Mat partPointMat;
		cap >> frame;
		if(frame.empty())
			break;

		double xAxis = i * rotationSpeed;
		double aAxis = -xAxis * tan(cutterPara.helixAngel * CV_PI / 180) / cutterPara.radius;
		vector<vector<double>> imagePoints;

		rotateTrans(xAxis, aAxis, inTransMat, outTransMat, RotateMat);
		getPartialPoints(cutterPara, xAxis, visualWidth, indexCenter,cutVecMat, partPointMat);
		pointsProject(partPointMat, inTransMat, outTransMat, RotateMat, imagePoints);

		double deltaWidth = imagePoints[indexCenter][0]-widthOffset;
		double deltaHeight = imagePoints[indexCenter][1]-heightOffset;
		//cout << indexCenter << " pianyi " << deltaWidth << endl;
		int num = imagePoints.size();
		//cout << "the " << i << " frame" << endl;

		for(int j=0;j<num-1;j++)
		{
			cv::Point Pt1 = Point(imagePoints[j][0]-deltaWidth, imagePoints[j][1]-deltaHeight);
			cv::Point Pt2 = Point(imagePoints[j+1][0]-deltaWidth, imagePoints[j+1][1]-deltaHeight);
			line(frame,Pt1,Pt2,cv::Scalar(255,255,0),3,8,0);
		}
		resize(frame,frame1,Size(frame.cols/resizeCount, frame.rows/resizeCount));
		imshow("blade", frame1);
		//sprintf(imageName,"%s%d%s","C:\\Users\\CGGI_006\\Desktop\\LXD\\image\\blade3",i,".bmp");
		//cv::imwrite(imageName,frame);

		if(cv::waitKey(30)>=0 || (cutterPara.cutEdgelen-xAxis)<rotationSpeed)
			break;
		i++;
	}
}

// 获得平角铣刀的轮廓线的有用的一部分
void getEdgePoints(
	double visualWidth,
	int& indexCenter,
	Parameter cutterPara,
	Mat& edgePoints
	)
{
	int pointsNum = int(2 * visualWidth / cutterPara.pointSteplen) + 1;
	int medianNum = int(visualWidth/cutterPara.pointSteplen);

	if(cutterPara.measurePos > visualWidth/2)
	{
		edgePoints = (Mat_<double>(3,1)<<0,-cutterPara.radius,0);
		for(int i=0;i<pointsNum;i++)
		{
			Mat colall = (Mat_<double>(3,1)<<-i*cutterPara.pointSteplen,-cutterPara.radius,0);
			cv::hconcat(edgePoints,colall,edgePoints);
		}
		indexCenter = medianNum;
	}

	else if(cutterPara.measurePos <= visualWidth/2)
	{
		edgePoints = (Mat_<double>(3,1)<<-visualWidth,-cutterPara.radius,0);
		for(int i=1;i<=medianNum;i++)
		{	
			Mat colPrev = (Mat_<double>(3,1)<< -visualWidth+i*cutterPara.pointSteplen,-cutterPara.radius,0);
			cv::hconcat(edgePoints,colPrev,edgePoints);
		}
		for(int i=medianNum+1;i<pointsNum;i++)
		{
			Mat colLater = (Mat_<double>(3,1)<<0,-cutterPara.radius + (i-medianNum) * cutterPara.pointSteplen,0);
			cv::hconcat(edgePoints,colLater,edgePoints);
		}
		indexCenter = int((visualWidth-cutterPara.measurePos)/cutterPara.pointSteplen);
	}
}

// 获得球头铣刀的轮廓线的有用的一部分
void getEdgeCirclePoints(
	double visualWidth,
	int& indexCenter,
	Parameter cutterPara,
	Mat& circlePoints
	)
{
	int pointsNum;
	if(cutterPara.measurePos >= cutterPara.radius + visualWidth / 2 
		&& cutterPara.upDown == 0)
	{
		pointsNum = int(visualWidth/cutterPara.pointSteplen) + 1;
		circlePoints = (Mat_<double>(3,1)<< -cutterPara.radius, -cutterPara.radius, 0);
		for(int i=1; i<pointsNum;i++)
		{
			Mat colall = (Mat_<double>(3,1)<< -cutterPara.radius - i*cutterPara.pointSteplen, -cutterPara.radius, 0);
			cv::hconcat(circlePoints, colall, circlePoints);
		}
		indexCenter = int(visualWidth/(cutterPara.pointSteplen*2));
	}
	else if(cutterPara.measurePos >= 0 
			&& cutterPara.measurePos < cutterPara.radius + visualWidth/2
			&& cutterPara.upDown == 0)
	{
		pointsNum = int((visualWidth+CV_PI)/cutterPara.pointSteplen) + 1;
		circlePoints = (Mat_<double>(3,1)<<-cutterPara.radius-visualWidth, -cutterPara.radius, 0);
		for(int i=1; i<visualWidth/cutterPara.pointSteplen; i++)
		{
			Mat colPrev = (Mat_<double>(3,1)<<-cutterPara.radius-visualWidth+i*cutterPara.pointSteplen, -cutterPara.radius, 0);
			cv::hconcat(circlePoints,colPrev,circlePoints);
		}
		for(int j=visualWidth/cutterPara.pointSteplen;j<pointsNum;j++)
		{
			Mat colLater = (Mat_<double>(3,1)<<cutterPara.radius*(-1+sin(j*cutterPara.pointSteplen-visualWidth)),
											   cutterPara.radius*(-cos(j*cutterPara.pointSteplen-visualWidth)),
											   0);
			cv::hconcat(circlePoints,colLater,circlePoints);
		}

		if(cutterPara.measurePos>cutterPara.radius)
			indexCenter = int((cutterPara.radius+visualWidth-cutterPara.measurePos)/cutterPara.pointSteplen);
		else
			indexCenter = int((visualWidth+asin((cutterPara.radius-cutterPara.measurePos)/cutterPara.radius))/cutterPara.pointSteplen);
	}
	
}

// 轮廓的指导线
void testEdgeVideo(
	double visualWidth,
	int indexCenter,
	char* fileName,
	Parameter cutterPara,
	Mat inTransMat,
	Mat outTransMat,
	Mat RotateMat,
	Mat edgePoints
	)
{
	VideoCapture cap(fileName);
	if(!cap.isOpened())
		cerr << "Can not open the file!" << endl;

	int i=0;
	cv::namedWindow("Edge",1);

	
	vector<vector<double>> imagePoints;
	double aAxis = 0;

	//getEdgePoints(visualWidth, indexCenter, cutterPara, edgePoints);
	//getEdgeCirclePoints(visualWidth, indexCenter, cutterPara, edgePoints);
	rotateTrans(cutterPara.measurePos, aAxis, inTransMat, outTransMat, RotateMat);
	pointsProject(edgePoints,inTransMat, outTransMat, RotateMat, imagePoints);
	double deltaWidth = imagePoints[indexCenter][0]-widthOffset;
	double deltaHeight = imagePoints[indexCenter][1]-heightOffset;
	int num = imagePoints.size();

	while(1)
	{
		Mat frame,frame1;
		cap >> frame;
		if(frame.empty())
			break;

		for(int j=0;j<num-1;j++)
		{
			cv::Point Pt1 = Point(imagePoints[j][0]-deltaWidth, imagePoints[j][1]-deltaHeight);
			cv::Point Pt2 = Point(imagePoints[j+1][0]-deltaWidth, imagePoints[j+1][1]-deltaHeight);
			line(frame,Pt1,Pt2,cv::Scalar(255,255,0),3,8,0);
		}
		resize(frame,frame1,Size(frame.cols/resizeCount, frame.rows/resizeCount));
		imshow("Edge", frame1);

		if(cv::waitKey(30)>=0)
			break;
		cout << ++i << "frame" << endl;

	}

}

// 指导线的测试程序
void testPro()
{
	Parameter cutterPara;
	cutterPara.cutterLen = 75;
	cutterPara.radius = 6;
	cutterPara.cutEdgelen = 24;
	cutterPara.taperAngel = 0;
	cutterPara.helixAngel = 45; 
	cutterPara.pointSteplen = 0.01;
	cutterPara.measurePos = 10;
	cutterPara.upDown = 0;							// 0表示上侧，1表示下侧

	double aAxisOffset = -0.35;
	double rotationSpeed = 0.06;
	double visualWidth = 3;
	int indexCenter = 0;

	resizeCount = 3;
	char* fileName;
	int proj = 1;							//1表示刃线指导线，2表示平角铣刀指导线，3表示球头铣刀指导线

	cv::Mat inTransMat, outTransMat, RotateMat;
	cv::Mat cutVecMat;
	cv::Mat edgePoints;

	switch(proj)
	{
	case 1:
		fileName = "data\\blade1.mp4";
		getWidthHeight(fileName);
		getPoint(cutterPara, aAxisOffset, cutVecMat);
		testBladeVideo(rotationSpeed, visualWidth, fileName, cutterPara, inTransMat, outTransMat, RotateMat, cutVecMat);
		break;
	case 2:
		fileName = "data\\edge1.avi";
		getWidthHeight(fileName);
		getEdgePoints(visualWidth, indexCenter, cutterPara, edgePoints);
		testEdgeVideo(visualWidth, indexCenter, fileName, cutterPara, inTransMat, outTransMat, RotateMat, edgePoints);
		break;
	case 3:
		fileName = "data\\head1.avi";
		cutterPara.measurePos = 0;
		getWidthHeight(fileName);
		getEdgeCirclePoints(visualWidth, indexCenter, cutterPara, edgePoints);
		testEdgeVideo(visualWidth,indexCenter, fileName, cutterPara, inTransMat, outTransMat, RotateMat, edgePoints);
		break;

	}
}

// 寻找指导线焦点的on_mouse函数
void on_mouse(
	int event,
	int x,
	int y,
	int flags,
	void* param)
{
	static cv::Point tmp_pts = cv::Point(0,0);
	switch(event)
	{
	case CV_EVENT_LBUTTONDOWN:
		tmp_pts = cv::Point(x,y);
		widthOffset = x*resizeCount;
		heightOffset = y*resizeCount;
		break;
	}
}