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
    p_task_scan_tower = new TaskScanTower();
}


vwpp::FlowController::~FlowController()
{
    delete p_task_takeoff;
    delete p_task_navigation;
    delete p_task_avoidance;
    delete p_task_hover_on_qr;
    delete p_task_delivering;
    delete p_task_landing;
    delete p_task_scan_tower;
}


int8_t vwpp::FlowController::run()
{

    if (cur_task_id == TAKEOFF)
    {
        p_task_takeoff->run();
        if (p_task_takeoff->getTaskState() == TASK_FINISH)
        {
            cur_task_id = NAVIGATION;
            // cur_task_id = SCANTOWER;
            ROS_INFO("Task switch to NAVIGATION!");
            // cur_task_id = LANDING;
        }
    }
    else if (cur_task_id == SCANTOWER)
    {
        p_task_scan_tower->run(DynamicRecfgInterface::getInstance()->getCycleMovingRadius());
    }
    else if (cur_task_id == NAVIGATION)
    {
        p_task_navigation->run();

        // Switch to Avoidance
        // if (VisionInterface::getInstance()->getYellowGateState())
        // {
        //     gate_type = YELLOW;
        //     cur_task_id = AVOIDANCE;
        //     p_task_avoidance->resetAdjustAltitudeOnXYYaw(PX4Interface::getInstance()->getCurX(),
        //                                                  PX4Interface::getInstance()->getCurY(),
        //                                                  PX4Interface::getInstance()->getCurYaw());
        //     ROS_INFO("Task switch to AVOIDANCE!");
        // }
        ROS_ERROR("Red gate state: %d", VisionInterface::getInstance()->getRedGateState());
        ROS_ERROR("Red gate depth: %lf", VisionInterface::getInstance()->getRedGateDepth());
        if (VisionInterface::getInstance()->getRedGateState())
        {
            gate_type = RED;
            cur_task_id = AVOIDANCE;
            p_task_avoidance->resetAdjustAltitudeOnXYYaw(PX4Interface::getInstance()->getCurX(),
                                                         PX4Interface::getInstance()->getCurY(),
                                                         PX4Interface::getInstance()->getCurYaw());
            ROS_INFO("Task switch to AVOIDANCE!");
        }
        if (p_task_navigation->getTaskState() == TASK_FINISH)
        {
            // Switch to HoverOnQR
            ROS_ERROR("Navigation finished!");
            if (VisionInterface::getInstance()->getGroundQRState())
            {
                p_task_navigation->restart();
                cur_task_id = HOVERONQR;
                last_qr_inform = VisionInterface::getInstance()->getGroundQRinform();
                p_task_hover_on_qr->resetRotatingOnXY(PX4Interface::getInstance()->getCurX(),
                                                      PX4Interface::getInstance()->getCurY());
                // cur_task_id = LANDING;
                ROS_INFO("Task switch to HOVERONQR!");
            }
        }
    }
    else if (cur_task_id == AVOIDANCE)
    {
        p_task_avoidance->run(gate_type);

        //     Switch to Navigation
        if (p_task_avoidance->getTaskState() == TASK_FINISH)
        {
            cur_task_id = NAVIGATION;
        }
    }
    else if (cur_task_id == HOVERONQR)
    {
        std::string cur_qr_inform = VisionInterface::getInstance()->getGroundQRinform();
        // TODO
        if (cur_qr_inform.empty() && last_qr_inform.empty())
        {
            cur_task_id = NAVIGATION;
            ROS_INFO("Task switch to NAVIGATION!");
        }
        else
        {
            static TaskID target_task_type_id = DELIVERING;

            if (cur_qr_inform.empty() && !last_qr_inform.empty())
            {
                cur_qr_inform = last_qr_inform;
            }
            task_type_id = p_task_hover_on_qr->run(target_task_type_id, cur_qr_inform);
            if (p_task_hover_on_qr->getTaskState() == TASK_FINISH)
            {

                ROS_INFO("task_type_id:%d and target_task_type_id:%d", task_type_id - '0', target_task_type_id + 1);

                if ((task_type_id - '0') == target_task_type_id + 1)
                {
                    if (target_task_type_id == DELIVERING)
                    {
                        cur_task_id = DELIVERING;
                        last_qr_inform = "";
                        ROS_INFO("Task switch to DELIVERING!");
                        target_task_type_id = LANDING;
                    }
                    else if (target_task_type_id == LANDING)
                    {
                        cur_task_id = LANDING;
                        last_qr_inform = "";
                        ROS_INFO("Task switch to LANDING!");
                    }
                }
                else
                {
                    cur_task_id = NAVIGATION;
                    last_qr_inform = "";
                    ROS_INFO("Task switch to NAVIGATION!");
                }

            }

        }
        last_qr_inform = cur_qr_inform;

    }
    else if (cur_task_id == DELIVERING)
    {
        p_task_delivering->run();

        if (p_task_delivering->getTaskState() == TASK_FINISH)
        {
            cur_task_id = NAVIGATION;
        }
    }
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


