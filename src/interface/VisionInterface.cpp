//
// Created by kehan on 19-7-16.
//

#include "interface/VisionInterface.h"


vwpp::VisionInterface::VisionInterface()
{
    nh = ros::NodeHandle();
    forward_camera = new MYCV(Forward_Camera, &nh);
    down_camera = new MYCV(Down_Camera, &nh);
}


vwpp::VisionInterface::VisionInterface(const VisionInterface &)
{
    //TODO ?
}


vwpp::VisionInterface &vwpp::VisionInterface::operator=(const VisionInterface &)
{
    //TODO ?
}


vwpp::VisionInterface::~VisionInterface()
{
    delete instance;
    delete down_camera;
    delete forward_camera;
}


int8_t vwpp::VisionInterface::update()
{
    try
    {
        down_camera->cvmain();
        // forward_camera->cvmain();
        cv::waitKey(1);

        return 0;
    }
    catch (std::exception &ex)   // TODO
    {
#ifdef TEST
        std::cout << ex.what() << std::endl;
#endif
        down_camera->destory();
        forward_camera->destory();
        return -1;
    }

}


vwpp::VisionInterface* vwpp::VisionInterface::instance = nullptr;
boost::mutex vwpp::VisionInterface::mutex_instance;


vwpp::VisionInterface* vwpp::VisionInterface::getInstance()
{
    // https://www.cnblogs.com/cxjchen/p/3148582.html
    if (instance == nullptr)
    {
        boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
        if (instance == nullptr)
        {
            instance = new VisionInterface();
        }
    }

    return instance;
}


std::string vwpp::VisionInterface::getGroundQRinform()
{
    return down_camera->QR_inform;
}


std::string vwpp::VisionInterface::getTownQRinform()
{
    return forward_camera->QR_inform;
}


double_t vwpp::VisionInterface::getLineOffset()
{
    return down_camera->line_dis;
}


double_t vwpp::VisionInterface::getLineRotation()
{
    return down_camera->line_rot;
}


bool vwpp::VisionInterface::getTownQRState()
{
    return forward_camera->detect_QR;
}


bool vwpp::VisionInterface::getGroundQRState()
{
    return down_camera->detect_QR;
}


double_t vwpp::VisionInterface::getQRxOffset()
{
    return down_camera->QR_location[0];
}


double_t vwpp::VisionInterface::getQRyOffset()
{
    return down_camera->QR_location[1];
}


bool vwpp::VisionInterface::getYellowGateState()
{
    return forward_camera->detect_yellow_gate;
}


double_t vwpp::VisionInterface::getYellowGateX()
{
    return forward_camera->yellow_gate_location[0];
}


double_t vwpp::VisionInterface::getYellowGateY()
{
    return forward_camera->yellow_gate_location[1];
}


double_t vwpp::VisionInterface::getYellowGateDepth()
{
    return forward_camera->yellow_gate_location[2];
}


bool vwpp::VisionInterface::getRedGateState()
{
    return false;
    return forward_camera->detect_red_gate;
}


double_t vwpp::VisionInterface::getRedGateX()
{
    return forward_camera->red_gate_location[0];
}


double_t vwpp::VisionInterface::getRedGateY()
{
    return forward_camera->red_gate_location[1];
}


double_t vwpp::VisionInterface::getRedGateDepth()
{
    return 20.0;
    return forward_camera->red_gate_location[2];
}


bool vwpp::VisionInterface::getBlueHState()
{
    return down_camera->detect_blueH;
}


bool vwpp::VisionInterface::getRedXState()
{
    return down_camera->detect_redX;
}


double_t vwpp::VisionInterface::getBlueHx()
{
    return down_camera->blueH_location[0];
}


double_t vwpp::VisionInterface::getBlueHy()
{
    return down_camera->blueH_location[1];
}


double_t vwpp::VisionInterface::getRedXx()
{
    return down_camera->redX_location[0];
}


double_t vwpp::VisionInterface::getRedXy()
{
    return down_camera->redX_location[1];
}


double_t vwpp::VisionInterface::getTownDepth()
{
    return forward_camera->tower_depth;
}




