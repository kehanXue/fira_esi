//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_TASK_H_
#define FIRA_ESI_TASK_H_

#include <cstdint>
#include <vector>

#include "fira_esi/vision/mycv.h"
#include "Action.h"
#include "VisionInterface.h"
#include "PX4Interface.h"

namespace vwpp
{
    enum TaskID
    {
        NAVIGATION = 0,
        AVOIDANCE,
        HOVERONQR,
        DELIVERING,
        SCANTOWER,
        SCANBUILDING,
        LANDING
    };

    enum TaskState
    {
        SUSPEND,
        START,
        PROCESSING = 0,
        FINISH,
    };

    class TaskBase
    {
    public:

        TaskBase();

        explicit TaskBase(TaskID _task_id);

        virtual ~TaskBase();

        TaskID task_id;
        TaskState task_state;

    private:
        std::vector<ActionID> vec_task_actionIDs;

        ros::NodeHandle nh;
    };

    class TaskNavigation
    {
    public:

        TaskNavigation();

        virtual ~TaskNavigation();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run();

    private:

        TaskBase* p_task_base;

        ActionID cur_action_id;

    };


    enum GateType
    {
        YELLOW = 0,
        RED
    };

    class TaskAvoidance
    {
    public:

        TaskAvoidance();

        virtual ~TaskAvoidance();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run(GateType _gate_type);

    private:

        TaskBase* p_task_base;

        ActionID cur_action_id;

        double_t altitude_target;

        // TODO Timer
        int64_t forward_counter;
    };

    class TaskHovering
    {
    public:

        TaskHovering();

        virtual ~TaskHovering();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run();

    private:
        TaskBase* p_task_base;

        ActionID cur_action_id;
    };
}


#endif //FIRA_ESI_TASK_H_

