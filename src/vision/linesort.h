#ifndef LINESORT_H
#define LINESORT_H

#include "vision/alldefine.h"
#include <opencv2/opencv.hpp>
#include <cmath>

#define Image_Width			640
#define Image_Height		480
//#define Image_Width		1920
//#define Image_Height		1080
#define	KP_Of_Translation	1.0
#define	Maxk				1000

class LineSort
{
public:
	LineSort();
	LineSort(double kk, double bb);
	LineSort(cv::Point p1, cv::Point p2);
	void 		setLine(cv::Point p1, cv::Point p2);
	void 		setLine(double kk, double bb);
	double	 	getdistopoint(cv::Point p);

public:
	double		k,b;
	cv::Point	A,B;

private:
	int			endx;
	void 		formatPoint();
	friend bool operator <(const LineSort &l1, const LineSort &l2)
	{ 
		return l1.endx > l2.endx;
	}
	friend bool operator >(const LineSort &l1, const LineSort &l2)
	{ 
		return l1.endx > l2.endx;
	}
};

#endif // LINESORT_H