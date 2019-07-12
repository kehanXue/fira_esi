//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_ACTION_H_
#define FIRA_ESI_ACTION_H_

#include <cstdint>

namespace vwpp
{
    enum ActionID
    {
        TrackingLine = 0,
        AdjustAltitude,
        Hovering,
        OpenClaw,
        CircularMotion
    };

    class Action
    {
    public:
        int8_t run();

        ActionID action_id;
    };
}


#endif //FIRA_ESI_ACTION_H_
