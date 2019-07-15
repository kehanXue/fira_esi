#ifndef LINEZXZ_H
#define LINEZXZ_H

#include "alldefine.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cmath>

#define		CutHeight			0
#define		KPOfTranslation		0.1
#define		KIOfTranslation		0.001
#define		Maxk				1000

class Linezxz
{
    //fabs(k)>0.0001 is wanted!
public:
	Linezxz();
	Linezxz(CvPoint x, CvPoint y);
	~Linezxz();

public:
	double		k,b;
	CvPoint		A,B;
	int			endx;
	void 		setLine(CvPoint x, CvPoint y);
	void 		setLine(double kk, double bb);
	double	 	getdistopoint(CvPoint p);

private:
	void 		formatPoint();
	void 		findendx();
	friend bool operator <(const Linezxz &aa, const Linezxz &bb)
	{ 
		return aa.endx > bb.endx;
	}
	friend bool operator >(const Linezxz &aa, const Linezxz &bb)
	{ 
		return aa.endx > bb.endx;
	}
};

#endif // LINEZXZ_H