//
// Created by kehan on 19-7-11.
//

#include <ros/ros.h>

#include "controller/FlowController.h"


int main(int argc, char** argv)
{
    // sleep(5);
    ros::init(argc, argv, "fira_esi_node");
    ros::NodeHandle nh("~");

    std::cout << "\033[32m" << ros::this_node::getName() << " start!"
              << "\033[0m" << std::endl;

    vwpp::DynamicRecfgInterface::getInstance()->update();


    vwpp::FlowController flow_controller;

    vwpp::PX4Interface::getInstance()->switchOffboard();
    vwpp::PX4Interface::getInstance()->unlockVehicle();


    ros::Rate loop_rate(10);
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

