//
// Created by kehan on 19-7-16.
//

#include "VisionInterface.h"


vwpp::VisionInterface::VisionInterface()
{
    nh = ros::NodeHandle("~");
    this->down_camera.init(DOWN_CAMERA, &nh);
    this->forward_camera.init(FORWARD_CAMERA, &nh);
}


vwpp::VisionInterface::VisionInterface(const VisionInterface &)
{

}


vwpp::VisionInterface &vwpp::VisionInterface::operator=(const VisionInterface &)
{

}


vwpp::VisionInterface::~VisionInterface()
{
    delete instance;
}


int8_t vwpp::VisionInterface::update()
{
    try
    {
        down_camera.cvmain();
        forward_camera.cvmain();

        return 0;
    }
    catch (Exception& ex)   // TODO
    {
        return -1;
    }

}


vwpp::VisionInterface *vwpp::VisionInterface::getInstance()
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
    return down_camera.QR_inform;
}


std::string vwpp::VisionInterface::getTownQRinform()
{
    return forward_camera.QR_inform;
}


double_t vwpp::VisionInterface::getLineOffset()
{
    return down_camera.line_dis;
}


double_t vwpp::VisionInterface::getLineRotation()
{
    return down_camera.line_rot;
}


double_t vwpp::VisionInterface::getQRxOffset()
{
    // TODO
    return 0;
}


double_t vwpp::VisionInterface::getQRyOffset()
{
    // TODO
    return 0;
}


bool vwpp::VisionInterface::getYellowGateState()
{
    return forward_camera.detect_yellow_gate;
}


double_t vwpp::VisionInterface::getYellowGateX()
{
    return forward_camera.yellow_gate_location[0];
}


double_t vwpp::VisionInterface::getYellowGateY()
{
    return forward_camera.yellow_gate_location[1];
}


double_t vwpp::VisionInterface::getYellowGateDepth()
{
    return forward_camera.yellow_gate_location[2];
}


bool vwpp::VisionInterface::getRedGateState()
{
    return forward_camera.detect_red_gate;
}


double_t vwpp::VisionInterface::getRedGateX()
{
    return forward_camera.red_gate_location[0];
}


double_t vwpp::VisionInterface::getRedGateY()
{
    return forward_camera.red_gate_location[1];
}


double_t vwpp::VisionInterface::getRedGateDepth()
{
    return forward_camera.red_gate_location[2];
}


bool vwpp::VisionInterface::getBlueHState()
{
    return forward_camera.detect_blueH;
}


bool vwpp::VisionInterface::getRedXState()
{
    return forward_camera.detect_redX;
}




