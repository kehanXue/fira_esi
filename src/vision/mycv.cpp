#include "vision/mycv.h"


MYCV::MYCV(int flag, ros::NodeHandle *pnh)
{
    param_topic = ros::this_node::getName()+"/";
#ifdef TEST
    printf("%s\n", param_topic.c_str());
#endif

    pnh->getParam(param_topic+"yamlpath", param_path);
#ifdef TEST
    printf("%s\n", param_path.c_str());
#endif

    camera_type = flag;
    nh = pnh;

    int camera_number = -1;

    if(flag == Down_Camera)
    {
        camera_number = getoneint("down_camera");
        vc.open(camera_number);
        //vc.set(CV_CAP_PROP_FRAME_WIDTH, 1280.0);
        //vc.set(CV_CAP_PROP_FRAME_HEIGHT, 720.0);
        //vc.set(CV_CAP_PROP_FRAME_WIDTH, 1920.0);
        //vc.set(CV_CAP_PROP_FRAME_HEIGHT, 1080.0);
        flag_findline       =   true;
        flag_findgate       =   false;
        flag_findQR         =   true;
        flag_findblueH      =   true;
        flag_findredX       =   true;
    }
    else if(flag == Forward_Camera)
    {
        sl::InitParameters zed_param;
        zed_param.camera_resolution         = sl::RESOLUTION_VGA;
        zed_param.camera_fps                = 30;
        zed_param.coordinate_units          = sl::UNIT_METER;
        zed_param.depth_mode                = sl::DEPTH_MODE_ULTRA;
        zed_param.depth_minimum_distance    = 0.3;
        if(sl::SUCCESS != zed.open(zed_param))
        {
#ifdef TEST
            printf("can't open zed\n");
#endif

            camera_number = getoneint("forward_camera");
            vc.open(camera_number);
        }

        flag_findline       =   false;
        flag_findgate       =   true;
        flag_findQR         =   true;
        flag_findblueH      =   false;
        flag_findredX       =   false;
    }
    else if(flag == No_Camera)
    {
        flag_findline       =   false;
        flag_findgate       =   false;
        flag_findQR         =   false;
        flag_findblueH      =   false;
        flag_findredX       =   false;
    }

    flag_first              =   true;
    flag_first_findlline    =   true;
    flag_first_findlgate    =   true;
    flag_first_findblueH    =   true;
    flag_first_findredX     =   true;

    line_dis                =   0.0;
    line_rot                =   0.0;
    detect_QR               =   false;
    QR_inform               =   "";
    QR_location[0]          =   0.0;
    QR_location[1]          =   0.0;
    detect_yellow_gate      =   false;
    detect_red_gate         =   false;
    yellow_gate_location[0] =   0.0;
    yellow_gate_location[1] =   0.0;
    yellow_gate_location[2] =   0.0;
    red_gate_location[0]    =   0.0;
    red_gate_location[1]    =   0.0;
    red_gate_location[2]    =   0.0;
    detect_blueH            =   false;
    detect_redX             =   false;
    blueH_location[0]       =   0.0;
    blueH_location[1]       =   0.0;
    redX_location[0]        =   0.0;
    redX_location[1]        =   0.0;
}

void MYCV::cvmain()
{
    cv::Mat image;

    if(camera_type == Down_Camera)
    {
        vc >> image;
    }
    else if(camera_type == Forward_Camera)
    {
        if(zed.isOpened())
        {
            zed.grab();
            sl::Mat zed_image;
            zed.retrieveImage(zed_image, sl::VIEW_LEFT);
            image = cv::Mat((int) zed_image.getHeight(), (int) zed_image.getWidth(),
                            CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU)).clone();
            cv::cvtColor(image, image, CV_BGRA2BGR);
            zed.retrieveMeasure(zed_depth, sl::MEASURE_DEPTH);
        }
        else
        {
            vc >> image;
        }
    }

    outimage = image.clone();

    if(flag_findline)
    {
        findline(image);
    }

    if(flag_findgate)
    {
        findgate(image);
    }

    if(flag_findQR)
    {
        findQR(image);
    }

    if(flag_findblueH)
    {
        findblueH(image);
    }

    if(flag_findredX)
    {
        findredX(image);
    }

#ifdef TEST
    if(flag_first)
    {
        flag_first = false;
        cv::namedWindow("VideoCapture image"+int2str(camera_type), 0);
    }
    cv::imshow("VideoCapture image"+int2str(camera_type), outimage);
#endif
}

void MYCV::findline(cv::Mat image)
{
    if(flag_first_findlline)
    {
        flag_first_findlline = false;
        line_threshold = getoneint("line_threshold");
    }

    static int x = line_threshold;
    if(x != line_threshold)
    {
        setoneint(line_threshold, "line_threshold");
        x = line_threshold;
    }

    Lineans.findmain(image, line_threshold, outimage);
    line_dis = Lineans.dis;
    line_rot = Lineans.rot;
}

void MYCV::findgate(cv::Mat image)
{
    if(flag_first_findlgate)
    {
        flag_first_findlgate = false;

#ifdef USE_HSV
        min_color[0] = getScalar("HSV_min_yellow");
        max_color[0] = getScalar("HSV_max_yellow");
        min_color[1] = getScalar("HSV_min_red");
        max_color[1] = getScalar("HSV_max_red");
#endif

#ifdef USE_BGR
        min_color[0] = getScalar("BGR_min_yellow");
        max_color[0] = getScalar("BGR_max_yellow");
        min_color[1] = getScalar("BGR_min_red");
        max_color[1] = getScalar("BGR_max_red");
#endif

#ifdef TEST
#ifdef FIND_GATE
        color_test();
#endif
#endif
    }

#ifdef TEST
#ifdef FIND_GATE
    check();
#endif
#endif

#ifdef USE_HSV
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
#endif

    cv::Point3d yellow, red;
    yellow = color_thing(image, min_color[0], max_color[0], "yellow");
    red = color_thing(image, min_color[1], max_color[1], "red");

    if(fabs(yellow.x+1.0) <1.0 && fabs(yellow.y+1.0) <1.0 )
    {
        detect_yellow_gate = false;
    }
    else
    {
        detect_yellow_gate = true;
        yellow_gate_location[0] = yellow.x;
        yellow_gate_location[1] = yellow.y;
        yellow_gate_location[2] = yellow.z;
        resetpoint(yellow_gate_location, image);
    }

    if(fabs(red.x+1.0) <1.0 && fabs(red.y+1.0) <1.0 )
    {
        detect_red_gate = false;
    }
    else
    {
        detect_red_gate = true;
        red_gate_location[0] = red.x;
        red_gate_location[1] = red.y;
        red_gate_location[2] = red.z;
        resetpoint(red_gate_location, image);
    }

}

void MYCV::findQR(cv::Mat image)
{
    cv::cvtColor(image, image, CV_BGR2GRAY);

    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    int width = image.cols;
    int height = image.rows;
    zbar::Image zbarimage(static_cast<unsigned int>(width), static_cast<unsigned int>(height),
            "Y800", image.data, static_cast<unsigned int>(width * height));

    scanner.scan(zbarimage);

    std::vector<std::string> inform;

    for(zbar::Image::SymbolIterator symbol = zbarimage.symbol_begin(); symbol != zbarimage.symbol_end(); ++symbol)
    {
        inform.push_back(symbol->get_data());

        int QR_X = 0;
        int QR_Y = 0;
        for(int i=0; i<symbol->get_location_size(); i++)
        {
            QR_X += symbol->get_location_x(i);
            QR_Y += symbol->get_location_y(i);
        }

        QR_location[0] = QR_X/4.0;
        QR_location[1] = QR_Y/4.0;

#ifdef TEST
        cv::circle(outimage, cv::Point(QR_location[0], QR_location[1]), 5, cv::Scalar(0,255,0), 3);
        cv::circle(outimage, cv::Point(QR_location[0], QR_location[1]), 20, cv::Scalar(0,255,0), 5);
#endif

        resetpoint(QR_location, image);
    }

    if(inform.empty())
    {
        detect_QR = false;
        QR_inform = "";
    }
    else if(fabs(QR_location[0])<170 && fabs(QR_location[1])<170)
    {
        detect_QR = true;
        QR_inform = inform[0];

        std::string ss(5, '0');
        ss[0] = QR_inform[0];
        ss[1] = QR_inform[2];
        ss[2] = QR_inform[4];
        ss[3] = QR_inform[6];
        ss[4] = QR_inform[8];

        QR_inform = ss;

#ifdef TEST
        cv::circle(outimage, cv::Point(QR_location[0], QR_location[1]), 3, cv::Scalar(0,255,255), 3);
#endif
    }
    else
    {
        detect_QR = false;
        QR_inform = "";
    }
}

void MYCV::findblueH(cv::Mat image)
{
    if(flag_first_findblueH)
    {
        flag_first_findblueH = false;

#ifdef USE_HSV
        blueH_min_color = getScalar("HSV_min_blueH");
        blueH_max_color = getScalar("HSV_max_blueH");
#endif

#ifdef USE_BGR
        blueH_min_color = getScalar("BGR_min_blueH");
        blueH_max_color = getScalar("BGR_max_blueH");
#endif

#ifdef TEST
#ifdef FIND_BLUEH
        color_test2();
        cv::namedWindow("blueH", 0);
#endif
#endif
    }

#ifdef TEST
#ifdef FIND_BLUEH
    check2();
#endif
#endif

#ifdef USE_HSV
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
#endif

    int sum_blue_rows = 0;
    int sum_blue_cols = 0;
    int number_blue = 0;

    cv::inRange(image, blueH_min_color, blueH_max_color, image);

    cv::Mat outimage_blueH(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

    for (int i = 0; i<image.rows; i++)
    {
        for (int j = 0; j<image.cols; j++)
        {
            //if (image.at<cv::Vec3b>(i,j)[0] > 70 &&
            //    image.at<cv::Vec3b>(i,j)[2] < 60)
            if(image.at<uchar>(i,j) == 255)
            {
#ifdef TEST
#ifdef FIND_BLUEH
                outimage_blueH.at<uchar>(i,j) = 255;
#endif
#endif

                sum_blue_rows += i;
                sum_blue_cols += j;
                number_blue++;
            }
        }
    }

#ifdef TEST
#ifdef FIND_BLUEH
    cv::imshow("blueH", outimage_blueH);
#endif
#endif

    if (number_blue>image.rows*image.cols/25)
    {
        detect_blueH = true;
        blueH_location[0] = 1.0* sum_blue_rows / number_blue;
        blueH_location[1] = 1.0* sum_blue_cols / number_blue;

#ifdef TEST
        cv::circle(outimage, cv::Point(blueH_location[0], blueH_location[1]), 5, cv::Scalar(255,0,0), 3);
        cv::circle(outimage, cv::Point(blueH_location[0], blueH_location[1]), 20, cv::Scalar(255,0,0), 5);
#endif
    }
    else
    {
        detect_blueH = false;
    }
}

void MYCV::findredX(cv::Mat image)
{
    if(flag_first_findredX)
    {
        flag_first_findredX = false;

#ifdef USE_HSV
        redX_min_color = getScalar("HSV_min_redX");
        redX_max_color = getScalar("HSV_max_redX");
#endif

#ifdef USE_BGR
        redX_min_color = getScalar("BGR_min_redX");
        redX_max_color = getScalar("BGR_max_redX");
#endif

#ifdef TEST
#ifdef FIND_REDX
        color_test3();
        cv::namedWindow("redX", 0);
#endif
#endif
    }

#ifdef TEST
#ifdef FIND_REDX
    check3();
#endif
#endif

#ifdef USE_HSV
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
#endif

    int sum_red_rows = 0;
    int sum_red_cols = 0;
    int number_red = 0;

    cv::inRange(image, redX_min_color, redX_max_color, image);

    cv::Mat outimage_redX(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            /*if (image.at<cv::Vec3b>(i,j)[0] < 75 &&
                image.at<cv::Vec3b>(i,j)[1] < 75 &&
                image.at<cv::Vec3b>(i,j)[2] > 110)*/
            if(image.at<uchar>(i,j) == 255)
            {
#ifdef TEST
#ifdef FIND_REDX
                outimage_redX.at<uchar>(i,j) = 255;
#endif
#endif

                sum_red_rows += i;
                sum_red_cols += j;
                number_red++;
            }

        }
    }

#ifdef TEST
#ifdef FIND_REDX
    cv::imshow("redX", outimage_redX);
#endif
#endif

    if (number_red>image.rows*image.cols/25)
    {
        detect_redX = true;
        redX_location[0] = 1.0* sum_red_rows / number_red;
        redX_location[1] = 1.0* sum_red_cols / number_red;

#ifdef TEST
        cv::circle(outimage, cv::Point(redX_location[0], redX_location[1]), 5, cv::Scalar(0,0,255), 3);
        cv::circle(outimage, cv::Point(redX_location[0], redX_location[1]), 20, cv::Scalar(0,0,255), 5);
#endif
    }
    else
    {
        detect_redX = false;
    }
}

cv::Point3d MYCV::color_thing(cv::Mat image, cv::Scalar min_color, cv::Scalar max_color, std::string name)
{
    cv::Mat thresholdimage;
    cv::inRange(image, min_color, max_color, thresholdimage);

    Proc_image(thresholdimage);

#ifdef TEST
#ifdef FIND_GATE
    static bool flag_first = true;
    static bool flag_second = true;
    if(flag_first || flag_second)
    {
        if(flag_first)
        {
            flag_first = false;
        }
        else
        {
            flag_second = false;
        }
        cv::namedWindow(name, 0);
    }
    cv::imshow(name, thresholdimage);
#endif
#endif

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(thresholdimage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > convexHulls(contours.size());
    for(unsigned int i = 0; i < contours.size(); i++)
    {
        cv::convexHull(contours[i], convexHulls[i]);
    }

    std::vector<cv::Rect> rect_array;
    for(unsigned int i = 0; i < convexHulls.size(); i++)
    {
        rect_array.push_back(cv::boundingRect(convexHulls[i]));
    }

    for(unsigned int i = 0; i < rect_array.size(); i++)
    {
        if(1.0*rect_array[i].area()/(image.cols*image.rows) > 0.05)
        {
#ifdef TEST
            if(name == "yellow")
            {
                cv::rectangle(outimage, rect_array[i], cv::Scalar(0,255,255), 3);
            }
            else if(name == "red")
            {
                cv::rectangle(outimage, rect_array[i], cv::Scalar(0,0,255), 3);
            }
#endif

            if(zed.isOpened())
            {
                float depth[8];
                int depth_x[8] = {rect_array[i].x+rect_array[i].width/9*1, rect_array[i].x+rect_array[i].width/9*2,
                                  rect_array[i].x+rect_array[i].width/9*3, rect_array[i].x+rect_array[i].width/9*4,
                                  rect_array[i].x+rect_array[i].width/9*5, rect_array[i].x+rect_array[i].width/9*6,
                                  rect_array[i].x+rect_array[i].width/9*7, rect_array[i].x+rect_array[i].width/9*8};
                int depth_y[8] = {rect_array[i].y+rect_array[i].height/10*1, rect_array[i].y+rect_array[i].height/10*1,
                                  rect_array[i].y+rect_array[i].height/10*1, rect_array[i].y+rect_array[i].height/10*1,
                                  rect_array[i].y+rect_array[i].height/10*1, rect_array[i].y+rect_array[i].height/10*1,
                                  rect_array[i].y+rect_array[i].height/10*1, rect_array[i].y+rect_array[i].height/10*1};
                for(int j=0; j<8; j++)
                {
                    zed_depth.getValue<float>(static_cast<unsigned int>(depth_x[j]),
                                              static_cast<unsigned int>(depth_y[j]), &depth[j], sl::MEM_CPU);
                }

                int depth_num = 0;
                float depth_ans = 0.0;
                for(int j=0; j<8; j++)
                {
                    if (std::isnormal(depth[j]))
                    {
                        depth_ans += depth[j];
                        depth_num++;
                    }
                }
                depth_ans /= depth_num;

                return cv::Point3d(rect_array[i].x+rect_array[i].width/2.0, rect_array[i].y+rect_array[i].height/2.0,
                                   depth_ans);
            }
            else
            {
                return cv::Point3d(rect_array[i].x+rect_array[i].width/2.0, rect_array[i].y+rect_array[i].height/2.0,
                                   image.cols/rect_array[i].width*0.75*sqrt(3.0));
            }
        }
    }

    return cv::Point3d(-1.0,-1.0,-1000.0);

    /* 120度相机 640*480分辨率 1.5m*1m门
     *     1.5m
     * ***********   1.3m
     *   *******
     *     ***
     *      *
     * 640 1.3m
     * 640/width*1.3
     */

}

void MYCV::Proc_image(cv::Mat &thresholdimage)
{
    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));

    int erodenumber = 1;
    int dilatenumber = 1;

    for(int i=0; i<erodenumber; i++)
    {
        cv::erode(thresholdimage,thresholdimage,erodeElement);
    }

    for(int i=0; i<dilatenumber; i++)
    {
        cv::dilate(thresholdimage,thresholdimage,dilateElement);
    }
}

void MYCV::resetpoint(double point[], cv::Mat image)
{
    cv::Point2d mid(point[0], point[1]);
    mid.x = (point[1] - image.rows/2.0);
    mid.y = (point[0] - image.cols/2.0);
    point[0] = mid.x;
    point[1] = mid.y;
}

static int x[24] = {0};

void MYCV::check()
{
    if(x[0] != static_cast<int>(min_color[0][0]) ||
       x[1] != static_cast<int>(min_color[0][1]) ||
       x[2] != static_cast<int>(min_color[0][2]) )
    {
#ifdef USE_HSV
        setScalar(cv::Scalar(x[0], x[1], x[2]), "HSV_min_yellow");
#endif
#ifdef USE_BGR
        setScalar(cv::Scalar(x[0], x[1], x[2]), "BGR_min_yellow");
#endif
        min_color[0] = cv::Scalar(x[0], x[1], x[2]);
    }

    if(x[3] != static_cast<int>(max_color[0][0]) ||
       x[4] != static_cast<int>(max_color[0][1]) ||
       x[5] != static_cast<int>(max_color[0][2]) )
    {
#ifdef USE_HSV
        setScalar(cv::Scalar(x[3], x[4], x[5]), "HSV_max_yellow");
#endif
#ifdef USE_BGR
        setScalar(cv::Scalar(x[3], x[4], x[5]), "BGR_max_yellow");
#endif
        max_color[0] = cv::Scalar(x[3], x[4], x[5]);
    }

    if(x[6] != static_cast<int>(min_color[1][0]) ||
       x[7] != static_cast<int>(min_color[1][1]) ||
       x[8] != static_cast<int>(min_color[1][2]) )
    {
#ifdef USE_HSV
        setScalar(cv::Scalar(x[6], x[7], x[8]), "HSV_min_red");
#endif
#ifdef USE_BGR
        setScalar(cv::Scalar(x[6], x[7], x[8]), "BGR_min_red");
#endif
        min_color[1] = cv::Scalar(x[6], x[7], x[8]);
    }

    if(x[9] != static_cast<int>(max_color[1][0]) ||
       x[10] != static_cast<int>(max_color[1][1]) ||
       x[11] != static_cast<int>(max_color[1][2]) )
    {
#ifdef USE_HSV
        setScalar(cv::Scalar(x[9], x[10], x[11]), "HSV_max_red");
#endif
#ifdef USE_BGR
        setScalar(cv::Scalar(x[9], x[10], x[11]), "BGR_max_red");
#endif
        max_color[1] = cv::Scalar(x[9], x[10], x[11]);
    }
}

void MYCV::color_test()
{
    x[0] = static_cast<int>(min_color[0][0]);
    x[1] = static_cast<int>(min_color[0][1]);
    x[2] = static_cast<int>(min_color[0][2]);
    x[3] = static_cast<int>(max_color[0][0]);
    x[4] = static_cast<int>(max_color[0][1]);
    x[5] = static_cast<int>(max_color[0][2]);
    x[6] = static_cast<int>(min_color[1][0]);
    x[7] = static_cast<int>(min_color[1][1]);
    x[8] = static_cast<int>(min_color[1][2]);
    x[9] = static_cast<int>(max_color[1][0]);
    x[10] = static_cast<int>(max_color[1][1]);
    x[11] = static_cast<int>(max_color[1][2]);
    cv::namedWindow("yellow", 0);
    cv::namedWindow("red", 0);

#ifdef USE_HSV
    cv::createTrackbar("yellow H1", "yellow", &x[0], 180, nullptr);
    cv::createTrackbar("yellow S1", "yellow", &x[1], 255, nullptr);
    cv::createTrackbar("yellow V1", "yellow", &x[2], 255, nullptr);
    cv::createTrackbar("yellow H2", "yellow", &x[3], 180, nullptr);
    cv::createTrackbar("yellow S2", "yellow", &x[4], 255, nullptr);
    cv::createTrackbar("yellow V2", "yellow", &x[5], 255, nullptr);
    cv::createTrackbar("red H1", "red", &x[6], 180, nullptr);
    cv::createTrackbar("red S1", "red", &x[7], 255, nullptr);
    cv::createTrackbar("red V1", "red", &x[8], 255, nullptr);
    cv::createTrackbar("red H2", "red", &x[9], 180, nullptr);
    cv::createTrackbar("red S2", "red", &x[10], 255, nullptr);
    cv::createTrackbar("red V2", "red", &x[11], 255, nullptr);
#endif

#ifdef USE_BGR
    cv::createTrackbar("yellow B1", "yellow", &x[0], 255, nullptr);
    cv::createTrackbar("yellow G1", "yellow", &x[1], 255, nullptr);
    cv::createTrackbar("yellow R1", "yellow", &x[2], 255, nullptr);
    cv::createTrackbar("yellow B2", "yellow", &x[3], 255, nullptr);
    cv::createTrackbar("yellow G2", "yellow", &x[4], 255, nullptr);
    cv::createTrackbar("yellow R2", "yellow", &x[5], 255, nullptr);
    cv::createTrackbar("red B1", "red", &x[6], 255, nullptr);
    cv::createTrackbar("red G1", "red", &x[7], 255, nullptr);
    cv::createTrackbar("red R1", "red", &x[8], 255, nullptr);
    cv::createTrackbar("red B2", "red", &x[9], 255, nullptr);
    cv::createTrackbar("red G2", "red", &x[10], 255, nullptr);
    cv::createTrackbar("red R2", "red", &x[11], 255, nullptr);
#endif
}

void MYCV::check2()
{
    if(x[12] != static_cast<int>(blueH_min_color[0]) ||
       x[13] != static_cast<int>(blueH_min_color[1]) ||
       x[14] != static_cast<int>(blueH_min_color[2]) )
    {
#ifdef USE_HSV
        setScalar(cv::Scalar(x[12], x[13], x[14]), "HSV_min_blueH");
#endif
#ifdef USE_BGR
        setScalar(cv::Scalar(x[12], x[13], x[14]), "BGR_min_blueH");
#endif
        blueH_min_color = cv::Scalar(x[12], x[13], x[14]);
    }

    if(x[15] != static_cast<int>(blueH_max_color[0]) ||
       x[16] != static_cast<int>(blueH_max_color[1]) ||
       x[17] != static_cast<int>(blueH_max_color[2]) )
    {
#ifdef USE_HSV
        setScalar(cv::Scalar(x[15], x[16], x[17]), "HSV_max_blueH");
#endif
#ifdef USE_BGR
        setScalar(cv::Scalar(x[15], x[16], x[17]), "BGR_max_blueH");
#endif
        blueH_max_color = cv::Scalar(x[15], x[16], x[17]);
    }
}

void MYCV::color_test2()
{
    x[12] = static_cast<int>(blueH_min_color[0]);
    x[13] = static_cast<int>(blueH_min_color[1]);
    x[14] = static_cast<int>(blueH_min_color[2]);
    x[15] = static_cast<int>(blueH_max_color[0]);
    x[16] = static_cast<int>(blueH_max_color[1]);
    x[17] = static_cast<int>(blueH_max_color[2]);
    cv::namedWindow("blueH", 0);

#ifdef USE_HSV
    cv::createTrackbar("blueH H1", "blueH", &x[12], 180, nullptr);
    cv::createTrackbar("blueH S1", "blueH", &x[13], 255, nullptr);
    cv::createTrackbar("blueH V1", "blueH", &x[14], 255, nullptr);
    cv::createTrackbar("blueH H2", "blueH", &x[15], 180, nullptr);
    cv::createTrackbar("blueH S2", "blueH", &x[16], 255, nullptr);
    cv::createTrackbar("blueH V2", "blueH", &x[17], 255, nullptr);
#endif

#ifdef USE_BGR
    cv::createTrackbar("blueH B1", "blueH", &x[12], 255, nullptr);
    cv::createTrackbar("blueH G1", "blueH", &x[13], 255, nullptr);
    cv::createTrackbar("blueH R1", "blueH", &x[14], 255, nullptr);
    cv::createTrackbar("blueH B2", "blueH", &x[15], 255, nullptr);
    cv::createTrackbar("blueH G2", "blueH", &x[16], 255, nullptr);
    cv::createTrackbar("blueH R2", "blueH", &x[17], 255, nullptr);
#endif
}

void MYCV::check3()
{
    if(x[18] != static_cast<int>(redX_min_color[0]) ||
       x[19] != static_cast<int>(redX_min_color[1]) ||
       x[20] != static_cast<int>(redX_min_color[2]) )
    {
#ifdef USE_HSV
        setScalar(cv::Scalar(x[18], x[19], x[20]), "HSV_min_redX");
#endif
#ifdef USE_BGR
        setScalar(cv::Scalar(x[18], x[19], x[20]), "BGR_min_redX");
#endif
        redX_min_color = cv::Scalar(x[18], x[19], x[20]);
    }

    if(x[21] != static_cast<int>(redX_max_color[0]) ||
       x[22] != static_cast<int>(redX_max_color[1]) ||
       x[23] != static_cast<int>(redX_max_color[2]) )
    {
#ifdef USE_HSV
        setScalar(cv::Scalar(x[21], x[22], x[23]), "HSV_max_redX");
#endif
#ifdef USE_BGR
        setScalar(cv::Scalar(x[21], x[22], x[23]), "BGR_max_redX");
#endif
        redX_max_color = cv::Scalar(x[21], x[22], x[23]);
    }
}

void MYCV::color_test3()
{
    x[18] = static_cast<int>(redX_min_color[0]);
    x[19] = static_cast<int>(redX_min_color[1]);
    x[20] = static_cast<int>(redX_min_color[2]);
    x[21] = static_cast<int>(redX_max_color[0]);
    x[22] = static_cast<int>(redX_max_color[1]);
    x[23] = static_cast<int>(redX_max_color[2]);
    cv::namedWindow("redX", 0);

#ifdef USE_HSV
    cv::createTrackbar("redX H1", "redX", &x[18], 180, nullptr);
    cv::createTrackbar("redX S1", "redX", &x[19], 255, nullptr);
    cv::createTrackbar("redX V1", "redX", &x[20], 255, nullptr);
    cv::createTrackbar("redX H2", "redX", &x[21], 180, nullptr);
    cv::createTrackbar("redX S2", "redX", &x[22], 255, nullptr);
    cv::createTrackbar("redX V2", "redX", &x[23], 255, nullptr);
#endif

#ifdef USE_BGR
    cv::createTrackbar("redX B1", "redX", &x[18], 255, nullptr);
    cv::createTrackbar("redX G1", "redX", &x[19], 255, nullptr);
    cv::createTrackbar("redX R1", "redX", &x[20], 255, nullptr);
    cv::createTrackbar("redX B2", "redX", &x[21], 255, nullptr);
    cv::createTrackbar("redX G2", "redX", &x[22], 255, nullptr);
    cv::createTrackbar("redX R2", "redX", &x[23], 255, nullptr);
#endif
}

std::string MYCV::int2str(int val)
{
    std::ostringstream out;
    out<<val;
    return out.str();
}

int MYCV::getoneint(std::string name)
{
    int x=0;

    nh->getParam(param_topic+name, x);

    return x;
}

void MYCV::setoneint(int x, std::string name)
{    YAML::Node yamlConfig = YAML::LoadFile(param_path);
    yamlConfig[name] = x;
    std::ofstream file;
    file.open(param_path.c_str());
    if(!file)
    {
        ROS_INFO("ERROR TO OPEN FILE %s", param_path.c_str());
    }
    file.flush();
    file << yamlConfig;
    file.close();
}

cv::Scalar MYCV::getScalar(std::string name)
{
    double x1=0.0, x2=0.0, x3=0.0;

    nh->getParam(param_topic+name+"1", x1);
    nh->getParam(param_topic+name+"2", x2);
    nh->getParam(param_topic+name+"3", x3);

    return cv::Scalar(x1,x2,x3);
}

void MYCV::setScalar(cv::Scalar color, std::string name)
{
    YAML::Node yamlConfig = YAML::LoadFile(param_path);
    yamlConfig[name+"1"] = color[0];
    yamlConfig[name+"2"] = color[1];
    yamlConfig[name+"3"] = color[2];
    std::ofstream file;
    file.open(param_path.c_str());
    if(!file)
    {
        ROS_INFO("ERROR TO OPEN FILE %s", param_path.c_str());
    }
    file.flush();
    file << yamlConfig;
    file.close();
}

void MYCV::destory()
{
    vc.release();
    zed.close();
    cv::destroyAllWindows();
}

void MYCV::open_findline()
{
    flag_findline = true;
}

void MYCV::close_findline()
{
    flag_findline = false;
}

void MYCV::open_findgate()
{
    flag_findgate = true;
}

void MYCV::close_findgate()
{
    flag_findgate = false;
}

void MYCV::open_findQR()
{
    flag_findQR = true;
}


void MYCV::close_findQR()
{
    flag_findQR = false;
}

void MYCV::open_findblueH()
{
    flag_findblueH = true;
}

void MYCV::close_findblueH()
{
    flag_findblueH = false;
}

void MYCV::open_findredX()
{
    flag_findredX = true;
}

void MYCV::close_findredX()
{
    flag_findredX = false;
}