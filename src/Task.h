//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_TASK_H_
#define FIRA_ESI_TASK_H_

#include <cstdint>
#include <vector>

#include "fira_esi/vision/"
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

        TaskID task_id;

    private:
        std::vector<Action> vec_task_actions;
    };
}


#endif //FIRA_ESI_TASK_H_

