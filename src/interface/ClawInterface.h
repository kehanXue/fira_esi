//
// Created by kehan on 2019/8/3.
//

#ifndef FIRA_ESI_CLAWINTERFACE_H_
#define FIRA_ESI_CLAWINTERFACE_H_


#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <boost/thread.hpp>

namespace vwpp
{

    class ClawInterface
    {
    public:

        static ClawInterface* getInstance();

        virtual ~ClawInterface();

        int8_t publishOpenClawMsg(const std_msgs::Bool &_msg);


    private:

        ClawInterface();
        ClawInterface(const ClawInterface &);
        ClawInterface &operator=(const ClawInterface &);

        static ClawInterface* instance;
        static boost::mutex mutex_instance;

        ros::NodeHandle nh;

        ros::Publisher open_claw_pub;
    };

}

#endif //FIRA_ESI_CLAWINTERFACE_H_
