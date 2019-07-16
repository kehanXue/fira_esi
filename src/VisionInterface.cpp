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


