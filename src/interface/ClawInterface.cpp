//
// Created by kehan on 2019/8/3.
//

#include "ClawInterface.h"


vwpp::ClawInterface::ClawInterface() :
        nh("~")
{
    open_claw_pub = nh.advertise<std_msgs::Bool>
            ("/MG995_key", 2);
}


vwpp::ClawInterface::ClawInterface(const vwpp::ClawInterface &)
{

}


vwpp::ClawInterface &vwpp::ClawInterface::operator=(const vwpp::ClawInterface &)
{

}


vwpp::ClawInterface* vwpp::ClawInterface::instance = nullptr;

boost::mutex vwpp::ClawInterface::mutex_instance;


vwpp::ClawInterface* vwpp::ClawInterface::getInstance()
{
    if (instance == nullptr)
    {
        boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
        if (instance == nullptr)
        {
            instance = new ClawInterface();
        }
    }

    return instance;
}


vwpp::ClawInterface::~ClawInterface()
{
    delete instance;
}


int8_t vwpp::ClawInterface::publishOpenClawMsg(const std_msgs::Bool &_msg)
{
    open_claw_pub.publish(_msg);

    return 0;
}





