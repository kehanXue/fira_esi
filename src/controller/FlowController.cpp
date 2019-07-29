//
// Created by kehan on 19-7-11.
//

#include "FlowController.h"


vwpp::FlowController::FlowController() :
        cur_task_id(TAKEOFF),
        // cur_task_id(NAVIGATION),
        gate_type(NONE),
        last_qr_inform("")
{

    cur_flow_state = FLOW_START;
    task_type_id = '0';

    // TODO shared_ptr
    p_task_takeoff = new TaskTakeoff();
    p_task_navigation = new TaskNavigation();
    p_task_avoidance = new TaskAvoidance();
    p_task_hover_on_qr = new TaskHoverOnQR();
    p_task_delivering = new TaskDelivering();
    p_task_landing = new TaskLanding();
}


vwpp::FlowController::~FlowController()
{
    delete p_task_takeoff;
    delete p_task_navigation;
    delete p_task_avoidance;
    delete p_task_hover_on_qr;
    delete p_task_delivering;
    delete p_task_landing;
}


int8_t vwpp::FlowController::run()
{

    if (cur_task_id == TAKEOFF)
    {
        p_task_takeoff->run();
        if (p_task_takeoff->getTaskState() == TASK_FINISH)
        {
            cur_task_id = NAVIGATION;
            ROS_INFO("Task switch to NAVIGATION!");
            // cur_task_id = LANDING;
        }
    }
    else if (cur_task_id == NAVIGATION)
    {
        p_task_navigation->run();

        // Switch to Avoidance
        // if (VisionInterface::getInstance()->getYellowGateState())
        // {
        //     gate_type = YELLOW;
        //     cur_task_id = AVOIDANCE;
        //     ROS_INFO("Task switch to AVOIDANCE!");
        // }
        // else if (VisionInterface::getInstance()->getRedGateState())
        // {
        //     gate_type = RED;
        //     cur_task_id = AVOIDANCE;
        //     ROS_INFO("Task switch to AVOIDANCE!");
        // }


        // Switch to HoverOnQR
        if (VisionInterface::getInstance()->getGroundQRState())
        {
            cur_task_id = HOVERONQR;
            // cur_task_id = LANDING;
            ROS_INFO("Task switch to HOVERONQR!");
        }

    }
        // else if (cur_task_id == AVOIDANCE)
        // {
        //     p_task_avoidance->run(gate_type);
        //
        //     Switch to Navigation
        // if (p_task_avoidance->getTaskState() == TASK_FINISH)
        // {
        //     cur_task_id = NAVIGATION;
        // }
        // }
    else if (cur_task_id == HOVERONQR)
    {
        std::string cur_qr_inform = VisionInterface::getInstance()->getGroundQRinform();
        if (cur_qr_inform.empty() && last_qr_inform.empty())
        {
            cur_task_id = NAVIGATION;
            ROS_INFO("Task switch to NAVIGATION!");
        }
        else
        {
            if (cur_qr_inform.empty() && !last_qr_inform.empty())
            {
                cur_qr_inform = last_qr_inform;
            }
            task_type_id = p_task_hover_on_qr->run(cur_task_id, cur_qr_inform);
            if (p_task_hover_on_qr->getTaskState() == TASK_FINISH)
            {

                static char target_task_type_id = '1';
                if (task_type_id == '0')
                {
                    cur_task_id = NAVIGATION;
                    last_qr_inform = "";
                    ROS_INFO("Task switch to NAVIGATION!");
                }
                else if (task_type_id == target_task_type_id)
                {
                    if (target_task_type_id == '1')
                    {
                        cur_task_id = DELIVERING;
                        last_qr_inform = "";
                        ROS_INFO("Task switch to DELIVERING!");
                        target_task_type_id = '4';
                    }
                    else if (target_task_type_id == '4')
                    {
                        cur_task_id = LANDING;
                        last_qr_inform = "";
                        ROS_INFO("Task switch to LANDING!");
                    }
                }

            }

        }
        last_qr_inform = cur_qr_inform;

    }
        // else if (cur_task_id == DELIVERING)
        // {
        //     p_task_delivering->run();
        //
        //     if (p_task_delivering->getTaskState() == TASK_FINISH)
        //     {
        //         cur_task_id = NAVIGATION;
        //     }
        // }
    else if (cur_task_id == LANDING)
    {
        p_task_landing->run();

        if (p_task_landing->getTaskState() == TASK_FINISH)
        {
            cur_flow_state = FLOW_FINISH;
            return 1;
        }
    }


    cur_flow_state = FLOW_PROCESSING;
    return 0;
}


vwpp::FlowState vwpp::FlowController::getFlowState()
{
    return cur_flow_state;
}


