//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_TASK_H_
#define FIRA_ESI_TASK_H_

#include <cstdint>
#include <vector>

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

    class Task
    {
    public:
        int8_t run();
        int8_t navigation();
        int8_t avoidance();
        int8_t hoverOnQR();
        int8_t delivering();
        int8_t scanTower();
        int8_t scanBuilding();
        int8_t landing();

        TaskID task_id;

    private:
        std::vector<Action> vec_task_actions;
    };
}


#endif //FIRA_ESI_TASK_H_

