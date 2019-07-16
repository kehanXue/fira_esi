//
// Created by kehan on 19-7-16.
//

#ifndef FIRA_ESI_VISIONINTERFACE_H_
#define FIRA_ESI_VISIONINTERFACE_H_

#include <boost/thread.hpp>
#include <ros/ros.h>

#include "fira_esi/vision/mycv.h"


// TODO Add thread or NOT
class VisionInterface
{
public:

    static VisionInterface *getInstance();

    virtual ~VisionInterface();

private:
    VisionInterface();

    VisionInterface(const VisionInterface &);

    VisionInterface &operator=(const VisionInterface &);


    static VisionInterface *instance;
    static boost::mutex mutex_instance;

    ros::NodeHandle nh;
    MYCV down_camera;
    MYCV forward_camera;
};



#endif //FIRA_ESI_VISIONINTERFACE_H_

