//
// Created by kehan on 19-7-16.
//

#include "VisionInterface.h"


VisionInterface::VisionInterface()
{
    nh = ros::NodeHandle("~");
    this->down_camera.init(DOWN_CAMERA, &nh);
    this->forward_camera.init(FORWARD_CAMERA, &nh);
}


VisionInterface::VisionInterface(const VisionInterface &)
{

}


VisionInterface &VisionInterface::operator=(const VisionInterface &)
{

}


VisionInterface::~VisionInterface()
{

}


int8_t VisionInterface::update()
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


VisionInterface *VisionInterface::getInstance()
{
    //
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


std::string VisionInterface::getGroundQRinform()
{
    return down_camera.QR_inform;
}


std::string VisionInterface::getTownQRinform()
{
    return forward_camera.QR_inform;
}


double_t VisionInterface::getLineOffset()
{
    return down_camera.line_dis;
}


double_t VisionInterface::getLineRotation()
{
    return down_camera.line_rot;
}


double_t VisionInterface::getQRxOffset()
{
    // TODO
    return 0;
}


double_t VisionInterface::getQRyOffset()
{
    // TODO
    return 0;
}


bool VisionInterface::getYellowGateState()
{
    return forward_camera.detect_yellow_gate;
}


double_t VisionInterface::getYellowGateX()
{
    return forward_camera.yellow_gate_location[0];
}


double_t VisionInterface::getYellowGateY()
{
    return forward_camera.yellow_gate_location[1];
}


double_t VisionInterface::getYellowGateDepth()
{
    return forward_camera.yellow_gate_location[2];
}


bool VisionInterface::getRedGateState()
{
    return forward_camera.detect_red_gate;
}


double_t VisionInterface::getRedGateX()
{
    return forward_camera.red_gate_location[0];
}


double_t VisionInterface::getRedGateY()
{
    return forward_camera.red_gate_location[1];
}


double_t VisionInterface::getRedGateDepth()
{
    return forward_camera.red_gate_location[2];
}


bool VisionInterface::getBlueHState()
{
    return forward_camera.detect_blueH;
}


bool VisionInterface::getRedXState()
{
    return forward_camera.detect_redX;
}




