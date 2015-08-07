#include <iostream>
#include "tooltaper.h"

int main()
{
	Parameter cutterPara;
	cutterPara.cutterLen = 75;
	cutterPara.diameter = 12;
	cutterPara.cutEdgelen = 24;
	cutterPara.taperAngel = 0;
	cutterPara.helixAngel = 45; 
	cutterPara.pointSteplen = 0.1;

	double aa[3][3]={{1,2,3},{4,5,6},{7,8,9}};
	cv::Mat cutline_image_vector;
 	getPoint(cutterPara, *aa, cutline_image_vector, "cutline_image_vector.txt");

	cv::Mat contour_image_vector;
	contourLine(cutterPara, *aa, contour_image_vector, "contourLine_vector.txt");

	return 0;
}