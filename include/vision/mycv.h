#ifndef MYCV_H
#define MYCV_H

#include "alldefine.h"
#include "findlines.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <math.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <zbar.h>

enum CameraNumber
{
    NO_CAMERA = -1,
    DOWN_CAMERA = 0,
    FORWARD_CAMERA = 1
};

class MYCV
{
public:
    MYCV();

    void init(int flag, ros::NodeHandle *pnh);

    void cvmain();

    void destory();

    ros::NodeHandle *nh;

public:
    void open_camera(int number);

    void open_findline();

    void close_findline();

    void open_findgate();

    void close_findgate();

    void open_findQR();

    void close_findQR();

    void open_findblueH();

    void close_findblueH();

    void open_findredX();

    void close_findredX();

public:
    double line_dis;
    double line_rot;
    std::string QR_inform;
    bool detect_yellow_gate;
    bool detect_red_gate;
    double yellow_gate_location[3];
    double red_gate_location[3];
    bool detect_blueH;
    bool detect_redX;
    double blueH_location[2];
    double redX_location[2];

private:
    bool flag_findline;
    bool flag_findgate;
    bool flag_findQR;
    bool flag_findblueH;
    bool flag_findredX;

private:
    int camera_type;
    cv::VideoCapture vc;
    FindLines Lineans;
    cv::Mat outimage;
    int line_threshold;
    //yellow red BGR
    cv::Scalar min_color[2], max_color[2];

private:
    void findline(cv::Mat image);

    void findgate(cv::Mat image);

    void findQR(cv::Mat image);

    void findblueH(cv::Mat image);

    void findredX(cv::Mat image);

    cv::Point3d color_thing(cv::Mat image, cv::Scalar min_color, cv::Scalar max_color, std::string name);

    void Proc_image(cv::Mat &thresholdimage);

    void resetpoint(double point[], cv::Mat image);

    void check();

    void color_test();

private:
    std::string int2str(int val);

    int getoneint(std::string name);

    void setoneint(int x, std::string name);

    cv::Scalar getScalar(std::string name);

    void setScalar(cv::Scalar color, std::string name);
};

#endif // MYCV_H