//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_ACTION_H_
#define FIRA_ESI_ACTION_H_

#include <cstdint>
#include <vector>
#include <cmath>
#include <tf/transform_listener.h>

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

    enum ActionID
    {
        TRACKINGLINE = 0,
        ADJUSTALTITUDE,
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

    struct Linear3D
    {
        double_t x;
        double_t y;
        double_t z;
    };

    class Action
    {
    public:

        static Action* getInstance();

        virtual ~Action();

        Velocity2D trackingLine(double_t _cur_line_y, double_t _target_yaw, double_t _cur_yaw,
                                double_t _forward_vel = DynamicRecfgInterface::getInstance()->getForwardVel());

        Linear3D adjustAltitude(double_t _target_altitude, double_t _cur_altitude, double_t _cur_x, double_t _cur_y);

        // Velocity2D hovering(double_t _cur_x, double_t _cur_y, double_t _target_yaw, double_t _cur_yaw);

        /* The _cur_x and _cur_y should be relative! Because the target is 0.! */
        Velocity2D hovering(double_t _cur_x, double_t _cur_y);

        Velocity2D rotating(Direction _direction, double_t _cur_yaw);

        Velocity2D rotating(double_t _target_yaw, double_t _cur_yaw);

        int8_t openClaw();

        // int8_t run(ActionID _action_id);


    private:

        Action();

        Action(const Action &);

        Action &operator=(const Action &);

        static Action* instance;
        static boost::mutex mutex_instance;


        tf::TransformListener odom_base_tf_listener;


    };

}

#endif //FIRA_ESI_ACTION_H_

