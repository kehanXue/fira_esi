//
// Created by kehan on 19-7-27.
//

#ifndef FIRA_ESI_ACTION_H_
#define FIRA_ESI_ACTION_H_


#include <cmath>
#include <tf/transform_listener.h>
#include "interface/DynamicRecfgInterface.h"
#include "interface/PX4Interface.h"
#include "controller/PIDController.h"
#include "utils/utils.h"


enum ActionID
{
    TRACKINGLINE = 0,
    ADJUSTALTITUDE,
    HOVERING,
    ROTATION,
    OPENCLAW,
    CIRCULARMOTION  //TODO
};

struct DroneVelocity
{
    double_t x;
    double_t y;
    double_t z;
    double_t yaw;
};


class ActionBase
{
public:
    ActionBase();

    virtual ~ActionBase();

    ActionID action_id;

private:

};

class ActionTrackingLine
{
public:

    explicit ActionTrackingLine(double_t _target_altitude);

    virtual ~ActionTrackingLine();

    ActionID getActionID();

    DroneVelocity calculateVelocity(double_t _cur_line_v_y, double_t _cur_v_yaw,
                                    double_t _forward_vel = vwpp::DynamicRecfgInterface::getInstance()->getForwardVel());

private:

    double_t target_altitude;

    // TODO ptr
    tf::TransformListener odom_base_tf_listener;

    ActionID action_id;
};


class ActionAdjustAltitude
{
public:

    ActionAdjustAltitude();

    virtual ~ActionAdjustAltitude();

    ActionID getActionId() const;

    DroneVelocity calculateVelocity(double_t _target_altitude, double_t _cur_altitude);

private:

    double_t initial_p_x;
    double_t initial_p_y;
    double_t initial_p_yaw;

    ActionID action_id;

};


class ActionHovering
{
public:

    explicit ActionHovering(double_t _target_altitude);

    virtual ~ActionHovering();

    ActionID getActionId() const;

    DroneVelocity calculateVelocity(double_t _cur_v_x, double_t _cur_v_y);

private:

    double_t target_altitude;
    double_t target_yaw;

    tf::TransformListener odom_base_tf_listener;

    ActionID action_id;
};

class ActionRotating
{
public:

    explicit ActionRotating(double_t _target_altitude);

    virtual ~ActionRotating();

    ActionID getActionId() const;

    DroneVelocity calculateVelocity(double_t _target_yaw, double_t _cur_yaw);

private:

    double_t target_altitude;

    // TODO Add to Constructor?
    double_t initial_p_x;
    double_t initial_p_y;

    ActionID action_id;
};

#endif //FIRA_ESI_ACTION_H_
