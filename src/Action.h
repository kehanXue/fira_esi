//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_ACTION_H_
#define FIRA_ESI_ACTION_H_

#include <cstdint>
#include <vector>
#include <cmath>
#include <tf/transform_listener.h>

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
        ROTATION,
        OPENCLAW,
        CIRCULARMOTION  //TODO
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

        Action();

        virtual ~Action();

        Velocity2D trackingLine(double_t _cur_line_y, double_t _target_yaw, double_t _cur_yaw,
                                double_t _forward_vel = 0.10);

        VelocityZ adjustAltitude(double_t _target_altitude, double_t _cur_altitude);

        Velocity2D hovering(double_t _cur_x, double_t _cur_y, double_t _target_yaw, double_t _cur_yaw);

        Velocity2D rotating(Direction _direction, double_t _cur_yaw);

        int8_t openClaw();

        int8_t run(ActionID _action_id);


    private:

        tf::TransformListener odom_base_tf_listener;


    };

}

#endif //FIRA_ESI_ACTION_H_

