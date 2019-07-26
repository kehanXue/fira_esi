//
// Created by kehan on 19-7-16.
//

#ifndef FIRA_ESI_VISIONINTERFACE_H_
#define FIRA_ESI_VISIONINTERFACE_H_

#include <boost/thread.hpp>
#include <ros/ros.h>

#include "vision/mycv.h"

namespace vwpp
{
    // TODO Add thread or NOT
    class VisionInterface
    {
    public:

        static VisionInterface* getInstance();

        virtual ~VisionInterface();

        int8_t update();

        std::string getGroundQRinform();

        std::string getTownQRinform();

        double_t getLineOffset();

        double_t getLineRotation();

        bool getQRState();

        double_t getQRxOffset();

        double_t getQRyOffset();

        bool getYellowGateState();

        double_t getYellowGateX();

        double_t getYellowGateY();

        double_t getYellowGateDepth();

        bool getRedGateState();

        double_t getRedGateX();

        double_t getRedGateY();

        double_t getRedGateDepth();

        bool getBlueHState();

        bool getBlueHx();

        bool getBlueHy();

        bool getRedXState();

        bool getRedXx();

        bool getRedXy();



    private:
        VisionInterface();

        VisionInterface(const VisionInterface &);

        VisionInterface &operator=(const VisionInterface &);


        static VisionInterface* instance;
        static boost::mutex mutex_instance;

        ros::NodeHandle nh;
        MYCV down_camera;
        MYCV forward_camera;
    };

}



#endif //FIRA_ESI_VISIONINTERFACE_H_

