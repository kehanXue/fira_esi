#ifndef FINDLINES_H
#define FINDLINES_H

#include "vision/alldefine.h"
#include "vision/linezxz.h"
#include <opencv2/opencv.hpp>
#include <algorithm>

class FindLines
{
public:
	FindLines();

public:
    double					dis;
    double					rot;
    void 	findmain(cv::Mat tpf, int &line_threshold, cv::Mat &outimage);
    bool					heng;
    bool					shu;

private:
    cv::Mat				    pFrame;
    cv::Mat 				pCutFrame;
    cv::Mat	    			pCutFrImg;
    std::vector<Linezxz>	ans;
    std::vector<cv::Vec4f>	lines;
    Linezxz					aim;
    Linezxz					aime;
    int 					litknum;
	int						nFrmNum;
	void	initaim();
	Linezxz finde(cv::Mat &image);
	void	aimchange();
	void	outputans();
};

#endif // FINDLINES_H