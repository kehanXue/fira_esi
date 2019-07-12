//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_ACTION_H_
#define FIRA_ESI_ACTION_H_

#include <cstdint>
#include <vector>
#include <cmath>

namespace vwpp
{

    enum Direction
    {
        LOCAL_FORWARD = 0,
        LOCAL_LEFT,
        LOCAL_BACK,
        LOCAL_RIGHT
    };

    enum ActionID
    {
        TRACKINGLINE = 0,
        AUJUSTALTITUDE,
        HOVERING,
        OPENCLAW,
        CIRCULARMOTION
    };

    struct Velocity2D
    {
        double_t x;
        double_t y;
        double_t yaw;
    };

    typedef double_t VelocityZ;

    class Action
    {
    public:

        Velocity2D trackingLine(double_t _cur_line_y, double_t _cur_yaw, Direction _direction,
                                double_t _forward_vel = 0.10);

        VelocityZ adjustAltitude(double_t _target_altitude, double_t _cur_altitude);

        Velocity2D hovering(double_t _cur_x, double_t _cur_y, double_t _cur_yaw);

        int8_t openClaw();

        ActionID action_id;
    };

}

#endif //FIRA_ESI_ACTION_H_

