//
// Created by kehan on 19-7-11.
//

#include "FlowController.h"


vwpp::FlowController::FlowController() :
        cur_task_id(TAKEOFF),
        gate_type(NONE)
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
        }
    }
    else if (cur_task_id == NAVIGATION)
    {
        p_task_navigation->run();

        // Switch to Avoidance
        if (VisionInterface::getInstance()->getYellowGateState())
        {
            gate_type = YELLOW;
            cur_task_id = AVOIDANCE;
        }
        else if (VisionInterface::getInstance()->getRedGateState())
        {
            gate_type = RED;
            cur_task_id = AVOIDANCE;
        }


        // Switch to HoverOnQR
        if (VisionInterface::getInstance()->getQRState())
        {
            cur_task_id = HOVERONQR;
        }

    }
    else if (cur_task_id == AVOIDANCE)
    {
        p_task_avoidance->run(gate_type);

        // Switch to Navigation
        if (p_task_avoidance->getTaskState() == TASK_FINISH)
        {
            cur_task_id = NAVIGATION;
        }
    }
    else if (cur_task_id == HOVERONQR)
    {
        task_type_id = p_task_hover_on_qr->run(cur_task_id);

        if (p_task_hover_on_qr->getTaskState() == TASK_FINISH)
        {

            switch (task_type_id)
            {
                case '0':
                    // Switch to Navigation
                    cur_task_id = NAVIGATION;
                    break;
                case '1':
                    // Switch to Delivering
                    cur_task_id = DELIVERING;
                case '4':
                    // Switch to Landing
                    cur_task_id = LANDING;
            }


        }
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


