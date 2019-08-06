//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_FLOWCONTROLLER_H_
#define FIRA_ESI_FLOWCONTROLLER_H_

#include "controller/Task.h"

namespace vwpp
{

    enum FlowState
    {
        FLOW_START,
        FLOW_PROCESSING,
        FLOW_FINISH
    };

    class FlowController
    {
    public:

        FlowController();

        virtual ~FlowController();

        int8_t run();

        FlowState getFlowState();


    private:

        TaskID cur_task_id;
        FlowState cur_flow_state;
        std::string last_qr_inform;

        TaskTakeoff* p_task_takeoff;
        TaskNavigation* p_task_navigation;
        TaskAvoidance* p_task_avoidance;
        TaskHoverOnQR* p_task_hover_on_qr;
        TaskDelivering* p_task_delivering;
        TaskLanding* p_task_landing;
        TaskScanTower* p_task_scan_tower;
        TaskScanBuilding* p_task_scan_building;

        GateType gate_type;
        char task_type_id;

    };
}


#endif //FIRA_ESI_FLOWCONTROLLER_H
