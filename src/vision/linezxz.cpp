#include "vision/linezxz.h"

Linezxz::Linezxz()
{
	k =	0;
	b =	0;
}

Linezxz::Linezxz(cv::Point x, cv::Point y)
{
	setLine(x,y);
}

void Linezxz::setLine(double kk, double bb)
{
	k =	kk;
	b =	bb;
	formatPoint();
}

void Linezxz::setLine(cv::Point x, cv::Point y)
{
	A =	x;
	B =	y;
	if(abs(A.x-B.x) <= 1)
	{
		k = Maxk;
	}
	else
	{
		k =	((x.y - y.y)*1.0 / (x.x - y.x));
	}
	
	b =	x.y - k*x.x;
	formatPoint();
}

void Linezxz::formatPoint()
{
	A.y	= 1;
	A.x	= static_cast<int>((1-b)/k);
	B.y	= 480-CutHeight;
	B.x	= static_cast<int>(((480-CutHeight)-b)/k);

	if(0 > A.x)
	{
		A.x = 0;
		A.y = static_cast<int>(b);
	}
	else if(A.x > 640)
	{
		A.x = 640;
		A.y = static_cast<int>(k * 640 + b);
	}

	if(0 > B.x)
	{
		B.x = 0;
		B.y = static_cast<int>(b);
	}
	else if(B.x > 640)
	{
		B.x = 640;
		B.y = static_cast<int>(k * 640 + b);
	}

	findendx();

}

void Linezxz::findendx()
{
	endx = static_cast<int>(((480-CutHeight)-b)/k);
}

double Linezxz::getdistopoint(cv::Point p)
{
	double AA,BB,CC;
	AA = k;
	BB = -1.0;
	CC = b;
	return fabs(AA*p.x+BB*p.y+CC)/sqrt(AA*AA+BB*BB);
}
