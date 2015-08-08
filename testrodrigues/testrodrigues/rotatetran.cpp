#include "rotatetran.h"

using namespace std;
using namespace cv;

int widthOffset;
int heightOffset;
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
	Mat M = (Mat_<double>(3,3)<< 33636*3,0,2592/2+0.5,0,33636*3,1944/2+0.5,0,0,1);

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
		//cout << imagePoints[i][0] << " " << imagePoints[i][1] << endl;
	}
}

// 展示图像上的刃线轨迹
void showImage(
	double xAxis,
	Parameter cutterPara,
	vector<vector<double>> imagePoints
	)
{
	int num = imagePoints.size();
	int centerIndex = floor(xAxis/cutterPara.pointSteplen);

	double deltaWidth = imagePoints[centerIndex][0]-widthOffset;
	double deltaHeight = imagePoints[centerIndex][1]-heightOffset;


	cv::Mat image = imread("C:\\Users\\CGGI_006\\Desktop\\LXD\\image\\blade370.bmp",1);

	for(int j=0;j<num-1;j++)
	{
		cv::Point Pt1 = Point(imagePoints[j][0]-deltaWidth, imagePoints[j][1]-deltaHeight);
		cv::Point Pt2 = Point(imagePoints[j+1][0]-deltaWidth, imagePoints[j+1][1]-deltaHeight);
		line(image,Pt1,Pt2,cv::Scalar(255,255,0),3,8,0);
	}
	cv::imwrite("image.bmp", image);
	resize(image,image,Size(image.cols/3,image.rows/3));
	imshow("image",image);
	cv::waitKey(0); 
}

void readVideo()
{
	VideoCapture cap("C:\\Users\\CGGI_006\\Desktop\\LXD\\project\\files\\GONEVideo\\0708\\head1.avi");
	if(!cap.isOpened())
	{
		cerr << "Can not open the file!" << endl;
	}
	char imageName[50];
	int i=0;
	namedWindow("blade",1);
	//Mat frame;
	//cap >> frame;
	//if(frame.empty())
	//	cerr << "Can not read frame!" << endl;
	//imshow("blade", frame);

	//cv::waitKey(0);
	//imwrite("frame3.bmp",frame);

	for(;;)
	{
		Mat frame;
		cap >> frame;
		if(frame.empty())
			break;
		sprintf(imageName,"%s%d%s","C:\\Users\\CGGI_006\\Desktop\\LXD\\image\\head1",++i,".bmp");
		cv::imwrite(imageName,frame);
		resize(frame,frame,Size(frame.cols/3,frame.rows/3));
		imshow("blade", frame);

		cout << i++ << endl;
		if(cv::waitKey(30) >= 0)
			break;
	}
}

void testBladeVideo(
	double rotationSpeed,
	double visualWidth,
	Parameter cutterPara,
	Mat inTransMat,
	Mat outTransMat,
	Mat RotateMat,
	Mat cutVecMat
	)
{
	VideoCapture cap("data\\blade3.mp4");
	if(!cap.isOpened())
	{
		cerr << "Can not open the file!" << endl;
	}
	//long count = static_cast<long>(cap.get(CV_CAP_PROP_FRAME_COUNT));

	char imageName[50];
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
		double aAxis = -2 * xAxis * tan(cutterPara.helixAngel * CV_PI / 180) / cutterPara.radius;
		vector<vector<double>> imagePoints;

		if(i==0)
		{
			resize(frame,framehead,Size(frame.cols/3,frame.rows/3));
			imshow("blade",framehead);
			cv::setMouseCallback("blade",on_mouse,0);
			waitKey(0);
		}

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
		resize(frame,frame1,Size(frame.cols/3, frame.rows/3));
		imshow("blade", frame1);
		//sprintf(imageName,"%s%d%s","C:\\Users\\CGGI_006\\Desktop\\LXD\\image\\blade3",i,".bmp");
		//cv::imwrite(imageName,frame);

		if(cv::waitKey(30)>=0 || (cutterPara.cutEdgelen-xAxis)<rotationSpeed)
			break;
		i++;
	}
}

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

void testEdgeVideo(
	double visualWidth,
	int indexCenter,
	Parameter cutterPara,
	Mat inTransMat,
	Mat outTransMat,
	Mat RotateMat
	)
{
	VideoCapture cap("C:\\Users\\CGGI_006\\Desktop\\LXD\\project\\files\\GONEVideo\\0708\\head1.avi");
	if(!cap.isOpened())
		cerr << "Can not open the file!" << endl;

	int i=0;
	cv::namedWindow("Edge",1);

	Mat edgePoints;
	vector<vector<double>> imagePoints;
	double aAxis = 0;

	//getEdgePoints(visualWidth, indexCenter, cutterPara, edgePoints);
	getEdgeCirclePoints(visualWidth, indexCenter, cutterPara, edgePoints);
	rotateTrans(cutterPara.measurePos, aAxis, inTransMat, outTransMat, RotateMat);
	pointsProject(edgePoints,inTransMat, outTransMat, RotateMat, imagePoints);
	widthOffset = 1321;
	heightOffset = 1140;
	double deltaWidth = imagePoints[indexCenter][0]-widthOffset;
	double deltaHeight = imagePoints[indexCenter][1]-heightOffset;
	int num = imagePoints.size();

	for(;;)
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
		resize(frame,frame1,Size(frame.cols/3, frame.rows/3));
		imshow("Edge", frame1);

		if(cv::waitKey(30)>=0)
			break;
		cout << ++i << "frame" << endl;

	}

}

void testPicture(
	Parameter cutterPara,
	Mat inTransMat,
	Mat outTransMat,
	Mat RotateMat,
	Mat cutVecMat
	)
{
	double xAxis = 0;
	double aAxis = -2 * xAxis / cutterPara.radius;
	vector<vector<double>> imagePoints;

	rotateTrans(xAxis, aAxis, inTransMat, outTransMat, RotateMat);

	pointsProject(cutVecMat, inTransMat, outTransMat, RotateMat, imagePoints);
	showImage(xAxis, cutterPara, imagePoints);
}

// 小测试
void testPro()
{
	Parameter cutterPara;
	cutterPara.cutterLen = 75;
	cutterPara.radius = 6;
	cutterPara.cutEdgelen = 24;
	cutterPara.taperAngel = 0;
	cutterPara.helixAngel = 35; 
	cutterPara.pointSteplen = 0.1;
	cutterPara.measurePos = 0;
	cutterPara.upDown = 0;							// 0表示上侧，1表示下侧

	double aAxisOffset = -0.33;
	double rotationSpeed = 0.06;
	double visualWidth = 3;
	int indexCenter = 0;
	cv::Mat inTransMat, outTransMat, RotateMat;
	cv::Mat cutVecMat;

	//getPoint(cutterPara, aAxisOffset, cutVecMat);
	//testBladeVideo(rotationSpeed, visualWidth, cutterPara, inTransMat, outTransMat, RotateMat, cutVecMat);
	
	//testPicture(cutterPara,coordOffset,inTransMat,outTransMat,RotateMat,cutVecMat);
	
	testEdgeVideo(visualWidth,indexCenter, cutterPara, inTransMat, outTransMat, RotateMat);
	
}

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
		widthOffset = x*3;
		heightOffset = y*3;
		break;
	}
}