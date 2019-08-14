#include "vision/mycv.h"

static int line_x = 0;
static int x[24] = {0};

#ifdef TEST
#ifdef TEST_ROS
#ifdef TEST_ROS_DY
void callback(dynamic_reconfigure::vision_dynamic_reconfigureConfig &config)
{
    line_x = config.line_threshold;

#ifdef USE_HSV
    x[0] = config.HSV_min_yellow1;
    x[1] = config.HSV_min_yellow2;
    x[2] = config.HSV_min_yellow3;
    x[3] = config.HSV_max_yellow1;
    x[4] = config.HSV_max_yellow2;
    x[5] = config.HSV_max_yellow3;
    x[6] = config.HSV_min_red1;
    x[7] = config.HSV_min_red2;
    x[8] = config.HSV_min_red3;
    x[9] = config.HSV_max_red1;
    x[10] = config.HSV_max_red2;
    x[11] = config.HSV_max_red3;
    x[12] = config.HSV_min_blueH1;
    x[13] = config.HSV_min_blueH2;
    x[14] = config.HSV_min_blueH3;
    x[15] = config.HSV_max_blueH1;
    x[16] = config.HSV_max_blueH2;
    x[17] = config.HSV_max_blueH3;
    x[18] = config.HSV_min_redX1;
    x[19] = config.HSV_min_redX2;
    x[20] = config.HSV_min_redX3;
    x[21] = config.HSV_max_redX1;
    x[22] = config.HSV_max_redX2;
    x[23] = config.HSV_max_redX3;
#endif
#ifdef USE_BGR
    x[0] = config.BGR_min_yellow1;
    x[1] = config.BGR_min_yellow2;
    x[2] = config.BGR_min_yellow3;
    x[3] = config.BGR_max_yellow1;
    x[4] = config.BGR_max_yellow2;
    x[5] = config.BGR_max_yellow3;
    x[6] = config.BGR_min_red1;
    x[7] = config.BGR_min_red2;
    x[8] = config.BGR_min_red3;
    x[9] = config.BGR_max_red1;
    x[10] = config.BGR_max_red2;
    x[11] = config.BGR_max_red3;
    x[12] = config.BGR_min_blueH1;
    x[13] = config.BGR_min_blueH2;
    x[14] = config.BGR_min_blueH3;
    x[15] = config.BGR_max_blueH1;
    x[16] = config.BGR_max_blueH2;
    x[17] = config.BGR_max_blueH3;
    x[18] = config.BGR_min_redX1;
    x[19] = config.BGR_min_redX2;
    x[20] = config.BGR_min_redX3;
    x[21] = config.BGR_max_redX1;
    x[22] = config.BGR_max_redX2;
    x[23] = config.BGR_max_redX3;
#endif
}
#endif
#endif
#endif

cv::Mat zed_left_image, zed_depth_image;
bool flag_zed_left_image = false, flag_zed_depth_image=false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if(flag_zed_left_image)
    {
        return;
    }

    try
    {
        zed_left_image=cv_bridge::toCvShare(msg, "bgr8")->image;
        flag_zed_left_image = true;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if(flag_zed_depth_image)
    {
        return;
    }

    zed_depth_image = cv::Mat::zeros(720,1280,CV_32FC1);
    float* depths = (float*)(&msg->data[0]);
    for(int i=0; i<zed_depth_image.cols; i++)
        for(int j=0; j<zed_depth_image.rows; j++)
        {
            zed_depth_image.at<float>(j,i) = depths[i+msg->width*j];
        }
    flag_zed_depth_image = true;
}

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
        flag_findtower      =   false;
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
            printf("can't open zed directly\n");
#endif

            nh->getParam(ros::this_node::getName()+"/zed_left_topic", zed_left_topic);
            nh->getParam(ros::this_node::getName()+"/zed_depth_topic", zed_depth_topic);

            if(zed_left_topic.empty() || zed_depth_topic.empty())
            {
#ifdef TEST
                printf("can't open zed\n");
#endif
                camera_number = getoneint("forward_camera");
                vc.open(camera_number);
            }
            else
            {
                static image_transport::ImageTransport _it(*nh);
                it = &_it;
                sub_left = it->subscribe(zed_left_topic, 1, imageCallback);
                sub_depth = nh->subscribe(zed_depth_topic, 1, depthCallback);
            }
        }

        flag_findline       =   false;
        flag_findgate       =   true;
        flag_findQR         =   false;
        flag_findblueH      =   false;
        flag_findredX       =   false;
        flag_findtower      =   false;
    }
    else if(flag == No_Camera)
    {
        flag_findline       =   false;
        flag_findgate       =   false;
        flag_findQR         =   false;
        flag_findblueH      =   false;
        flag_findredX       =   false;
        flag_findtower      =   false;
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
    detect_tower            =   false;
    tower_depth             =   0.0;

#ifdef TEST
    #ifdef TEST_ROS
    line_x = getoneint("line_threshold");
#ifdef USE_HSV
    x[0] = getoneint("HSV_min_yellow1");
    x[1] = getoneint("HSV_min_yellow2");
    x[2] = getoneint("HSV_min_yellow3");
    x[3] = getoneint("HSV_max_yellow1");
    x[4] = getoneint("HSV_max_yellow2");
    x[5] = getoneint("HSV_max_yellow3");
    x[6] = getoneint("HSV_min_red1");
    x[7] = getoneint("HSV_min_red2");
    x[8] = getoneint("HSV_min_red3");
    x[9] = getoneint("HSV_max_red1");
    x[10] = getoneint("HSV_max_red2");
    x[11] = getoneint("HSV_max_red3");
    x[12] = getoneint("HSV_min_blueH1");
    x[13] = getoneint("HSV_min_blueH2");
    x[14] = getoneint("HSV_min_blueH3");
    x[15] = getoneint("HSV_max_blueH1");
    x[16] = getoneint("HSV_max_blueH2");
    x[17] = getoneint("HSV_max_blueH3");
    x[18] = getoneint("HSV_min_redX1");
    x[19] = getoneint("HSV_min_redX2");
    x[20] = getoneint("HSV_min_redX3");
    x[21] = getoneint("HSV_max_redX1");
    x[22] = getoneint("HSV_max_redX2");
    x[23] = getoneint("HSV_max_redX3");
#endif
#ifdef USE_BGR
    x[0] = getoneint("BGR_min_yellow1");
    x[1] = getoneint("BGR_min_yellow2");
    x[2] = getoneint("BGR_min_yellow3");
    x[3] = getoneint("BGR_max_yellow1");
    x[4] = getoneint("BGR_max_yellow2");
    x[5] = getoneint("BGR_max_yellow3");
    x[6] = getoneint("BGR_min_red1");
    x[7] = getoneint("BGR_min_red2");
    x[8] = getoneint("BGR_min_red3");
    x[9] = getoneint("BGR_max_red1");
    x[10] = getoneint("BGR_max_red2");
    x[11] = getoneint("BGR_max_red3");
    x[12] = getoneint("BGR_min_blueH1");
    x[13] = getoneint("BGR_min_blueH2");
    x[14] = getoneint("BGR_min_blueH3");
    x[15] = getoneint("BGR_max_blueH1");
    x[16] = getoneint("BGR_max_blueH2");
    x[17] = getoneint("BGR_max_blueH3");
    x[18] = getoneint("BGR_min_redX1");
    x[19] = getoneint("BGR_min_redX2");
    x[20] = getoneint("BGR_min_redX3");
    x[21] = getoneint("BGR_max_redX1");
    x[22] = getoneint("BGR_max_redX2");
    x[23] = getoneint("BGR_max_redX3");
#endif

#ifdef TEST_ROS_DY
    static bool test_ros_first = true;
    if(test_ros_first)
    {
        test_ros_first = false;
        server = new(dynamic_reconfigure::Server<dynamic_reconfigure::vision_dynamic_reconfigureConfig>);
        server_callback = new(dynamic_reconfigure::Server<dynamic_reconfigure::vision_dynamic_reconfigureConfig>::CallbackType);
        *server_callback = boost::bind(&callback, _1);
        server->setCallback(*server_callback);
    }
#endif
#endif
#endif
}

MYCV::~MYCV()
{
    destory();
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
        else if(vc.isOpened())
        {
            vc >> image;
        }
        else
        {
            if(flag_zed_left_image)
            {
                image = zed_left_image.clone();
                flag_zed_left_image = false;
            }

            if(flag_zed_depth_image)
            {
                image_depth = zed_depth_image.clone();
                flag_zed_depth_image = false;
            }
        }
    }


    if(image.empty() || (camera_type==Forward_Camera && !zed.isOpened() && !vc.isOpened() && image_depth.empty()))
    {
        return;
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

    if(flag_findtower)
    {
        findtower(image);
    }

#ifdef TEST
    #ifndef TEST_ROS
    if(flag_first)
    {
        flag_first = false;
        cv::namedWindow("VideoCapture image"+int2str(camera_type), 0);
    }
    cv::imshow("VideoCapture image"+int2str(camera_type), outimage);
#endif

#ifdef TEST_ROS
    static image_transport::ImageTransport it(*nh);
    static image_transport::Publisher pub0 = it.advertise("vision/out/downcamera", 1);
    static image_transport::Publisher pub1 = it.advertise("vision/out/forwardcamera", 1);
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();
    header.frame_id = "image"+int2str(camera_type);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", outimage).toImageMsg();
    if(camera_type == Down_Camera)
    {
        pub0.publish(msg);
    }
    else if(camera_type == Forward_Camera)
    {
        pub1.publish(msg);
    }
#endif
#endif
}

void MYCV::findline(cv::Mat image)
{
    if(flag_first_findlline)
    {
        flag_first_findlline = false;
        line_threshold = getoneint("line_threshold");
        line_x = line_threshold;
    }

#ifdef TEST
    #ifdef FIND_LINE
#ifndef TEST_ROS
    if(line_x != line_threshold)
    {
        setoneint(line_threshold, "line_threshold");
        line_x = line_threshold;
    }
#endif

#ifdef TEST_ROS
    if(line_x != line_threshold)
    {
        setoneint(line_x, "line_threshold");
        line_threshold = line_x;
    }
#endif
#endif
#endif

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
        #ifndef TEST_ROS
#ifdef FIND_GATE
        color_test();
#endif
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

    static std::vector<bool> detect_yellow(5, false);
    for(unsigned int i=0; i<detect_yellow.size()-1; i++)
    {
        detect_yellow[i] = detect_yellow[i+1];
    }

    if(fabs(yellow.x+1.0) <1.0 && fabs(yellow.y+1.0) <1.0 )
    {
        detect_yellow_gate = false;
        detect_yellow[detect_yellow.size()-1] = detect_yellow_gate;
    }
    else
    {
        detect_yellow_gate = true;
        yellow_gate_location[0] = yellow.x;
        yellow_gate_location[1] = yellow.y;
        yellow_gate_location[2] = yellow.z;
        resetpoint(yellow_gate_location, image);

        detect_yellow[detect_yellow.size()-1] = detect_yellow_gate;
        int flag = true;
        for(unsigned int i=0; i<detect_yellow.size()-1; i++)
        {
            if(detect_yellow[i] == false)
            {
                flag = false;
                break;
            }
        }

        if(!(yellow_gate_location[2] < 1.5 && yellow_gate_location[2] > 0.8 && flag))
        {
            detect_yellow_gate = false;
        }
    }

    static std::vector<bool> detect_red(5, false);
    for(unsigned int i=0; i<detect_red.size()-1; i++)
    {
        detect_red[i] = detect_red[i+1];
    }

    if(fabs(red.x+1.0) <1.0 && fabs(red.y+1.0) <1.0 )
    {
        detect_red_gate = false;
        detect_red[detect_red.size()-1] = detect_red_gate;
    }
    else
    {
        detect_red_gate = true;
        red_gate_location[0] = red.x;
        red_gate_location[1] = red.y;
        red_gate_location[2] = red.z;
        resetpoint(red_gate_location, image);

        detect_red[detect_red.size()-1] = detect_red_gate;
        int flag = true;
        for(unsigned int i=0; i<detect_red.size()-1; i++)
        {
            if(detect_red[i] == false)
            {
                flag = false;
                break;
            }
        }

        if(!(red_gate_location[2] < 1.5 && red_gate_location[2] > 0.8 && flag))
        {
            detect_red_gate = false;
        }
    }

}

void MYCV::findQR(cv::Mat image)
{
    cv::cvtColor(image, image, CV_BGR2GRAY);

    if(camera_type == Forward_Camera && (image.cols!=672 || image.rows!=376))
    {
        cv::resize(image,image,cv::Size(672,376));
    }

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
        for(unsigned int i=0; i<symbol->get_location_size(); i++)
        {
            QR_X += symbol->get_location_x(i);
            QR_Y += symbol->get_location_y(i);
        }

        QR_location[0] = QR_X/4.0;
        QR_location[1] = QR_Y/4.0;

#ifdef TEST
        cv::circle(outimage, cv::Point2d(QR_location[0], QR_location[1]), 5, cv::Scalar(0,255,0), 3);
        cv::circle(outimage, cv::Point2d(QR_location[0], QR_location[1]), 20, cv::Scalar(0,255,0), 5);
#endif

        resetpoint(QR_location, image);
    }

    int QR_rect_size = 400;

    if(inform.empty())
    {
        detect_QR = false;
        QR_inform = "";
    }
    else if(camera_type == Down_Camera && fabs(QR_location[0])<QR_rect_size && fabs(QR_location[1])<QR_rect_size)
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
        cv::rectangle(outimage, cv::Point(outimage.cols/2-QR_rect_size, outimage.rows/2-QR_rect_size),
                cv::Point(outimage.cols/2+QR_rect_size, outimage.rows/2+QR_rect_size), cv::Scalar(255,255,0));
#endif
    }
    else if(camera_type == Forward_Camera && !inform.empty())
    {
        FILE* tower_inform = fopen("../tower_inform", "a+");
        for(unsigned int i=0; i<inform.size(); i++)
        {
            ROS_INFO("!!!!!!!!!!!!!TOWER_INFORM: %s",inform[i].c_str());
            fprintf(tower_inform, "%d.%d: %s\n", ros::Time::now().sec, ros::Time::now().nsec, inform[i].c_str());
        }
        fclose(tower_inform);
        QR_inform = inform[0];
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
        #ifndef TEST_ROS
#ifdef FIND_BLUEH
        color_test2();
        cv::namedWindow("blueH", 0);
#endif
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

    for(int i = 0; i<image.rows; i++)
    {
        for(int j = 0; j<image.cols; j++)
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
#ifndef TEST_ROS
    cv::imshow("blueH", outimage_blueH);
#endif

#ifdef TEST_ROS
    static image_transport::ImageTransport it(*nh);
    static image_transport::Publisher pub = it.advertise("vision/out/blueH", 1);
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();
    header.frame_id = "blueH";
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", outimage_blueH).toImageMsg();
    pub.publish(msg);
#endif
#endif
#endif

    if(number_blue>1000)
    {
        detect_blueH = true;
        blueH_location[1] = 1.0* sum_blue_rows / number_blue;
        blueH_location[0] = 1.0* sum_blue_cols / number_blue;

#ifdef TEST
        cv::circle(outimage, cv::Point2d(blueH_location[0], blueH_location[1]), 5, cv::Scalar(255,0,0), 3);
        cv::circle(outimage, cv::Point2d(blueH_location[0], blueH_location[1]), 20, cv::Scalar(255,0,0), 5);
#endif

        resetpoint(blueH_location, image);
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
        #ifndef TEST_ROS
#ifdef FIND_REDX
        color_test3();
        cv::namedWindow("redX", 0);
#endif
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

    for(int i = 0; i < image.rows; i++)
    {
        for(int j = 0; j < image.cols; j++)
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
#ifndef TEST_ROS
    cv::imshow("redX", outimage_redX);
#endif

#ifdef TEST_ROS
    static image_transport::ImageTransport it(*nh);
    static image_transport::Publisher pub = it.advertise("vision/out/redX", 1);
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();
    header.frame_id = "redX";
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", outimage_redX).toImageMsg();
    pub.publish(msg);
#endif
#endif
#endif

    if(number_red>1000)
    {
        detect_redX = true;
        redX_location[1] = 1.0* sum_red_rows / number_red;
        redX_location[0] = 1.0* sum_red_cols / number_red;

        redX_location[1] -= 50;
#ifdef TEST
        cv::circle(outimage, cv::Point2d(redX_location[0], redX_location[1]), 5, cv::Scalar(0,0,255), 3);
        cv::circle(outimage, cv::Point2d(redX_location[0], redX_location[1]), 20, cv::Scalar(0,0,255), 5);
#endif

        resetpoint(redX_location, image);

	    // if(detect_redX)
	    // {
		//     static int count = 0;
		//     if(count++ < 8)
		//     {
		// 	    detect_redX = false;
		//     }
	    // }
    }
    else
    {
        detect_redX = false;
    }
}

void MYCV::findtower(cv::Mat image)
{
    cv::Mat thresholdimage(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

    const static int threshold = 13;
    for(unsigned int i = 0; i < image.rows; i++)
    {
        for(unsigned int j = 0; j < image.cols; j++)
        {
            float depth = 0.0;
            if(zed.isOpened())
            {
                zed_depth.getValue<float>(j, i, &depth, sl::MEM_CPU);
            }


            if(abs(image.at<cv::Vec3b>(i,j)[0] - image.at<cv::Vec3b>(i,j)[1])<threshold &&
               abs(image.at<cv::Vec3b>(i,j)[2] - image.at<cv::Vec3b>(i,j)[1])<threshold &&
               abs(image.at<cv::Vec3b>(i,j)[0] - image.at<cv::Vec3b>(i,j)[2])<threshold &&
               std::isnormal(depth) && depth<2.0)
            {
                thresholdimage.at<uchar>(i,j) = 255;
            }
        }
    }

    Proc_image(thresholdimage);

#ifdef TEST
    #ifdef FIND_TOWER
#ifndef TEST_ROS
    static bool flag_first = true;
    if(flag_first)
    {
        flag_first = false;
        cv::namedWindow("tower", 0);
    }
    cv::imshow("tower", thresholdimage);
#endif

#ifdef TEST_ROS
    static image_transport::ImageTransport it(*nh);
    static image_transport::Publisher pub = it.advertise("vision/out/tower", 1);
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();
    header.frame_id = "tower";
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", thresholdimage).toImageMsg();
    pub.publish(msg);
#endif
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
        if(1.0*rect_array[i].area()/(image.cols*image.rows) > 0.15)
        {
            int number = 0;
            for(int j = rect_array[i].y; j < rect_array[i].y+rect_array[i].height; j++)
            {
                for(int k = rect_array[i].x; k < rect_array[i].x+rect_array[i].width; k++)
                {
                    if(thresholdimage.at<uchar>(j,k) == 255)
                    {
                        number++;
                    }
                }
            }

            if(number<0.3*rect_array[i].height*rect_array[i].width)
            {
                continue;
            }

#ifdef TEST
            cv::rectangle(outimage, rect_array[i], cv::Scalar(255,0,255), 3);
#endif
            detect_tower = true;

            if(zed.isOpened())
            {
                int depth_num = 0;
                float depth_ans = 0.0;
                for(unsigned int j = rect_array[i].y; j < rect_array[i].y+rect_array[i].height; j++)
                {
                    for(unsigned int k = rect_array[i].x; k < rect_array[i].x+rect_array[i].width; k++)
                    {
                        if(thresholdimage.at<uchar>(j,k) == 255)
                        {
                            float depth;
                            zed_depth.getValue<float>(k, j, &depth, sl::MEM_CPU);
                            if(std::isnormal(depth) && depth<2.0)
                            {
                                depth_ans += depth;
                                depth_num++;
                            }
                        }
                    }
                }

                depth_ans /= depth_num;
                if(depth_num < 100)
                {
                    detect_tower = false;
                    depth_ans = 5.0;
                }

                tower_depth = depth_ans;
            }
            else if(vc.isOpened())
            {
                tower_depth = 1.0*image.cols/rect_array[i].width*0.75*sqrt(3.0);
            }
            else
            {
                int depth_num = 0;
                float depth_ans = 0.0;
                for(unsigned int j = rect_array[i].y; j < rect_array[i].y+rect_array[i].height; j++)
                {
                    for(unsigned int k = rect_array[i].x; k < rect_array[i].x+rect_array[i].width; k++)
                    {
                        if(thresholdimage.at<uchar>(j,k) == 255)
                        {
                            float depth = image_depth.at<float>(j,k);
                            if(std::isnormal(depth) && depth>0.0 && depth<2.0)
                            {
                                depth_ans += depth;
                                depth_num++;
                            }
                        }
                    }
                }

                depth_ans /= depth_num;
                if(depth_num < 100)
                {
                    depth_ans = 20.0;
                }

                tower_depth = depth_ans;
            }
        }
    }
}

cv::Point3d MYCV::color_thing(cv::Mat image, cv::Scalar min_color, cv::Scalar max_color, std::string name)
{
    cv::Mat thresholdimage;
    cv::inRange(image, min_color, max_color, thresholdimage);

    Proc_image(thresholdimage);

#ifdef TEST
    #ifdef FIND_GATE
#ifndef TEST_ROS
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

#ifdef TEST_ROS
    static image_transport::ImageTransport it(*nh);
    static image_transport::Publisher pub = it.advertise("vision/out/yellow", 1);
    static image_transport::Publisher pub2 = it.advertise("vision/out/red", 1);
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();
    header.frame_id = name;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", thresholdimage).toImageMsg();
    if(name == "yellow")
    {
        pub.publish(msg);
    }
    else if(name == "red")
    {
        pub2.publish(msg);
    }
#endif
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
        if(1.0*rect_array[i].area()/(image.cols*image.rows) > 0.3)
        {
            int number = 0;
            for(int j = rect_array[i].y; j < rect_array[i].y+rect_array[i].height; j++)
            {
                for(int k = rect_array[i].x; k < rect_array[i].x+rect_array[i].width; k++)
                {
                    if(thresholdimage.at<uchar>(j,k) == 255)
                    {
                        number++;
                    }
                }
            }

            if(!(number>0.15*rect_array[i].height*rect_array[i].width &&
                 number<0.3*rect_array[i].height*rect_array[i].width))
            {
                continue;
            }

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
                int depth_num = 0;
                float depth_ans = 0.0;
                for(unsigned int j = rect_array[i].y; j < rect_array[i].y+rect_array[i].height; j++)
                {
                    for(unsigned int k = rect_array[i].x; k < rect_array[i].x+rect_array[i].width; k++)
                    {
                        if(thresholdimage.at<uchar>(j,k) == 255)
                        {
                            float depth;
                            zed_depth.getValue<float>(k, j, &depth, sl::MEM_CPU);
                            if(std::isnormal(depth) && depth>0.0 && depth<5.0)
                            {
                                depth_ans += depth;
                                depth_num++;
                            }
                        }
                    }
                }

                depth_ans /= depth_num;
                if(depth_num < 100)
                {
                    depth_ans = 20.0;
                }

                return cv::Point3d(rect_array[i].x+rect_array[i].width/2.0, rect_array[i].y+rect_array[i].height/2.0,
                                   depth_ans);
            }
            else if(vc.isOpened())
            {
                return cv::Point3d(rect_array[i].x+rect_array[i].width/2.0, rect_array[i].y+rect_array[i].height/2.0,
                                   1.0*image.cols/rect_array[i].width*0.75*sqrt(3.0));
            }
            else
            {
                int depth_num = 0;
                float depth_ans = 0.0;
                for(unsigned int j = rect_array[i].y; j < rect_array[i].y+rect_array[i].height; j++)
                {
                    for(unsigned int k = rect_array[i].x; k < rect_array[i].x+rect_array[i].width; k++)
                    {
                        if(thresholdimage.at<uchar>(j,k) == 255)
                        {
                            float depth = image_depth.at<float>(j,k);
                            if(std::isnormal(depth) && depth>0.0 && depth<5.0)
                            {
                                depth_ans += depth;
                                depth_num++;
                            }
                        }
                    }
                }

                depth_ans /= depth_num;
                if(depth_num < 100)
                {
                    depth_ans = 20.0;
                }

                return cv::Point3d(rect_array[i].x+rect_array[i].width/2.0, rect_array[i].y+rect_array[i].height/2.0,
                                   depth_ans);
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
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));

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

int MYCV::getoneint(std::string name)
{
    int x=0;

    nh->getParam(param_topic+name, x);

    return x;
}

cv::Scalar MYCV::getScalar(std::string name)
{
    int x1=0, x2=0, x3=0;

    nh->getParam(param_topic+name+"1", x1);
    nh->getParam(param_topic+name+"2", x2);
    nh->getParam(param_topic+name+"3", x3);

    return cv::Scalar(x1,x2,x3);
}

std::string MYCV::int2str(int val)
{
    std::ostringstream out;
    out<<val;
    return out.str();
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
    if(vc.isOpened())
    {
        vc.release();
    }
    if(zed.isOpened())
    {
        zed.close();
    }

    if(camera_type == Forward_Camera)
    {
        FILE* origin = fopen("../tower_inform", "r");
        FILE* result = fopen("../tower_inform_result", "w");

        int a,b;
        char s[100] = "", ss[100] = "";
        while(fscanf(origin, "%d.%d: %s", &a,&b,s) != EOF)
        {
            if(strcmp(s,ss) != 0)
            {
                ROS_INFO("!!!!!!!!!!!!!");
                ROS_INFO("%d.%d: %s", a,b,s);
                fprintf(result, "%d.%d: %s\n", a,b,s);
            }
            strcpy(ss,s);
        }

        fclose(origin);
        fclose(result);
    }

#ifdef TEST
    #ifndef TEST_ROS
    cv::destroyAllWindows();
#endif
#endif
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

void MYCV::open_findtower()
{
    flag_findtower = true;
}

void MYCV::close_findtower()
{
    flag_findtower = false;
}
