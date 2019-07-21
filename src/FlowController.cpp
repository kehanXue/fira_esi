//
// Created by kehan on 19-7-11.
//

#include "FlowController.h"


vwpp::FlowController::FlowController() :
        cur_task_id(TAKEOFF),
        gate_type(NONE)
{
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
        if (p_task_takeoff->getTaskState() == FINISH)
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
        if (p_task_avoidance->getTaskState() == FINISH)
        {
            cur_task_id = NAVIGATION;
        }
    }
    else if (cur_task_id == HOVERONQR)
    {
        p_task_hover_on_qr->run(cur_task_id);

        // Switch to Navigation
        if (p_task_hover_on_qr->getTaskState() == FINISH)
        {
            cur_task_id = NAVIGATION;
        }
    }



    return 0;
}
