//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_TASK_H_
#define FIRA_ESI_TASK_H_

#include <cstdint>
#include <vector>

#include "fira_esi/vision/mycv.h"
#include "Action.h"

namespace vwpp
{
    enum TaskID
    {
        Navigation = 0,
        Avoidance,
        HoverOnQR,
        Delivering,
        ScanTower,
        ScanBuilding,
        Landing
    };

    class TaskBase
    {
    public:

        TaskBase();

        virtual ~TaskBase();

        TaskID task_id;

    private:
        std::vector<Action> vec_task_actions;

        ros::NodeHandle nh;

        MYCV down_camera;
        MYCV forward_camera;
    };

    class TaskNavigation
    {
    public:
        TaskNavigation();

        virtual ~TaskNavigation();

        TaskID getTaskID();


    private:
        TaskBase task_base;

    };

    class TaskAvoidance
    {
    public:
        TaskAvoidance();

        virtual ~TaskAvoidance();

        TaskID getTaskID();

    private:
        TaskBase task_base;

    };
}


#endif //FIRA_ESI_TASK_H_

