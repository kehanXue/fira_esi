#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image=cv_bridge::toCvShare(msg, "bgr8")->image;

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
        }

        FILE* tower_inform = fopen("../tower_inform", "a+");
        for(unsigned int i=0; i<inform.size(); i++)
        {
            ROS_INFO("!!!!!!!!!!!!!TOWER_INFORM: %s",inform[i].c_str());
            fprintf(tower_inform, "%d.%d: %s\n", ros::Time::now().sec, ros::Time::now().nsec, inform[i].c_str());
        }
        fclose(tower_inform);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_qr_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/zed/zed_node/left/image_rect_color", 1, imageCallback);

    ros::spin();

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

    return 0;
}
