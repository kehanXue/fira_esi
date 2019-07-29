//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_TASK_H_
#define FIRA_ESI_TASK_H_

#include <cstdint>
#include <vector>

#include "vision/mycv.h"
#include "controller/Action.h"
#include "interface/VisionInterface.h"
#include "interface/PX4Interface.h"
#include "utils/JudgeAchieveCounter.h"
#include "interface/DynamicRecfgInterface.h"

namespace vwpp
{

    enum Direction
    {
        LOCAL_FORWARD = 0,
        LOCAL_LEFT,
        LOCAL_BACK,
        LOCAL_RIGHT
    };

    enum TaskID
    {
        TAKEOFF = -4,
        NAVIGATION = -3,
        AVOIDANCE = -2,
        HOVERONQR = -1,
        DELIVERING = 0,
        SCANTOWER = 1,
        SCANBUILDING = 2,
        LANDING = 3
    };

    enum TaskState
    {
        TASK_SUSPEND,
        TASK_START,
        TASK_PROCESSING = 0,
        TASK_FINISH
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

        ros::NodeHandle nh;
    };

    class TaskTakeoff
    {
    public:

        TaskTakeoff();

        virtual ~TaskTakeoff();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run();

    private:

        TaskBase* p_task_base;

        ActionID cur_action_id;

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
        RED,
        NONE
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

    class TaskHoverOnQR
    {
    public:

        TaskHoverOnQR();

        virtual ~TaskHoverOnQR();

        TaskID getTaskID();

        TaskState getTaskState();

        char run(TaskID _cur_task_id, std::string _qr_inform);

    private:
        TaskBase* p_task_base;

        // TODO Add to task_base
        ActionID cur_action_id;
    };

    class TaskDelivering
    {
    public:

        TaskDelivering();

        virtual ~TaskDelivering();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run();

    private:

        TaskBase* p_task_base;

        ActionID cur_action_id;
    };


    class TaskLanding
    {
    public:

        TaskLanding();

        virtual ~TaskLanding();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run();

    private:

        TaskBase* p_task_base;

        ActionID cur_action_id;

    };

}


#endif //FIRA_ESI_TASK_H_

