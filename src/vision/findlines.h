#ifndef FINDLINES_H
#define FINDLINES_H

#include "vision/alldefine.h"
#include "vision/linesort.h"
#include <opencv2/opencv.hpp>
#include <algorithm>

class FindLines
{
public:
	FindLines();

public:
    double					dis;
    double					rot;
    void 	                findmain(cv::Mat image, int &line_threshold, cv::Mat &outimage);

private:
	bool 					flag_first;
	bool					vertical;
    bool					transverse;
    std::vector<LineSort>	ans;
    LineSort				aim;
    LineSort				aime;
    LineSort                finde(cv::Mat &image);
	void	                aimchange();
	void	                outputans();
};

#endif // FINDLINES_H