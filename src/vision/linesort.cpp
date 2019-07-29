#include "vision/linesort.h"

LineSort::LineSort()
{
	k =	0;
	b =	0;
}

LineSort::LineSort(double kk, double bb)
{
	setLine(kk,bb);
}

LineSort::LineSort(cv::Point p1, cv::Point p2)
{
	setLine(p1,p2);
}

void LineSort::setLine(double kk, double bb)
{
	k =	kk;
	b =	bb;
	formatPoint();
}

void LineSort::setLine(cv::Point p1, cv::Point p2)
{
	if(abs(p1.x-p2.x) <= 1)
	{
		k = Maxk;
	}
	else
	{
		k =	((p1.y - p2.y)*1.0 / (p1.x - p2.x));
	}
	
	b =	p1.y - k*p1.x;
	formatPoint();
}

void LineSort::formatPoint()
{
	A.y	= 1;
	A.x	= static_cast<int>((1-b)/k);
	B.y	= Image_Height;
	B.x	= static_cast<int>((Image_Height-b)/k);

	if(A.x < 0)
	{
		A.x = 0;
		A.y = static_cast<int>(b);
	}
	else if(A.x > Image_Width)
	{
		A.x = Image_Width;
		A.y = static_cast<int>(k*Image_Width + b);
	}

	if(B.x < 0)
	{
		B.x = 0;
		B.y = static_cast<int>(b);
	}
	else if(B.x > Image_Width)
	{
		B.x = Image_Width;
		B.y = static_cast<int>(k*Image_Width + b);
	}

	endx = static_cast<int>((Image_Height-b)/k);
}

double LineSort::getdistopoint(cv::Point p)
{
	return -1.0*fabs(k*p.x -1.0*p.y +b) / sqrt(k*k + 1.0);
}
