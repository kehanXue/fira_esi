#ifndef FINDLINES_H
#define FINDLINES_H

#include "linezxz.h"
#include <vector>
#include <iostream>
#include <algorithm>  
#include <opencv2/opencv.hpp>
#include <cstdio>

using namespace std;
using namespace cv;

class FindLines
{
public:
	FindLines();

public:
	std::vector<Linezxz>	ans;
	IplImage*				pFrame;
	Mat						tpmat;
	Linezxz					aim;
	double					dis;
	int 					litknum;
	double					rot;
	void findmain(IplImage* tpf, int &line_threshold, cv::Mat &outimage);
	void destory();

private:
	int						nFrmNum;
	bool                	ort;
	IplImage*				pCutFrame;
	IplImage*				pCutFrImg;
	CvSeq*					lines;
	CvMemStorage*			storage;
	Linezxz					aime;
	void	initaim();
	Linezxz finde(cv::Mat &image);
	void	aimchange();
	void	outputans();
};

#endif // FINDLINES_H