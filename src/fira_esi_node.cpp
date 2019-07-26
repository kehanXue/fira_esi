//
// Created by kehan on 19-7-11.
//

#include <ros/ros.h>

#include "FlowController.h"


int main(int argc, char** argv)
{
    // TODO
    ros::init(argc, argv, "fira_esi_node");
    ros::NodeHandle nh("~");

    ros::Rate loop_rate(10);


    vwpp::PX4Interface::getInstance()->switchOffboard();
    vwpp::PX4Interface::getInstance()->unlockVehicle();

    vwpp::FlowController flow_controller;


    while (ros::ok())
    {
        flow_controller.run();
        if (flow_controller.getFlowState() == vwpp::FlowState::FLOW_FINISH)
        {
            break;
        }

        loop_rate.sleep();
    }

    return 0;
}

