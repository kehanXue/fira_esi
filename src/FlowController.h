//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_FLOWCONTROLLER_H_
#define FIRA_ESI_FLOWCONTROLLER_H_

#include "Task.h"

namespace vwpp
{
    class FlowController
    {
    public:

        FlowController();

        virtual ~FlowController();

        int8_t run();


    private:

        TaskID cur_task_id;

        TaskTakeoff* p_task_takeoff;
        TaskNavigation* p_task_navigation;
        TaskAvoidance* p_task_avoidance;
        TaskHoverOnQR* p_task_hover_on_qr;
        TaskDelivering* p_task_delivering;
        TaskLanding* p_task_landing;
        
        GateType gate_type;

    };
}


#endif //FIRA_ESI_FLOWCONTROLLER_H
