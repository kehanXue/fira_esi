//
// Created by kehan on 19-7-27.
//

#ifndef FIRA_ESI_ACTION_H_
#define FIRA_ESI_ACTION_H_


#include <cmath>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "interface/DynamicRecfgInterface.h"
#include "interface/PX4Interface.h"
#include "controller/PIDController.h"
#include "utils/utils.h"

namespace vwpp
{


    enum ActionID
    {
        TRACKINGLINE = 0,
        ADJUSTALTITUDE,
        HOVERING,
        ROTATION,
        OPENCLAW,
        CYCLEMOVING,
        GOTOPOSE,
        GOTOPOSITION            // Go to position hold yaw
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

        TargetVelXYPosZYaw calculateVelocity(double_t _cur_line_v_y, double_t _cur_v_yaw,
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

        TargetPosXYZYaw calculateVelocity(double_t _target_altitude);

        DroneVelocity calculateVelocity(double_t _target_altitude, double_t _cur_altitude);

        int8_t setAdjustAltitudeXYYaw(double_t _on_x, double_t _on_y, double_t _on_yaw);

    private:

        double_t on_p_x;
        double_t on_p_y;
        double_t on_p_yaw;

        ActionID action_id;

    };


    class ActionHovering
    {
    public:

        explicit ActionHovering(double_t _target_altitude);

        virtual ~ActionHovering();

        ActionID getActionId() const;

        TargetVelXYPosZYaw calculateVelocity(double_t _cur_v_x, double_t _cur_v_y);

        int8_t resetTargetYaw(double_t _target_yaw);

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

        TargetPosXYZYaw calculateVelocity(double_t _target_yaw);

        DroneVelocity calculateVelocity(double_t _target_yaw, double_t _cur_yaw);

        int8_t resetRotatingOnXY(double_t _hover_x, double_t _hover_y);

    private:

        double_t target_altitude;

        double_t on_p_x;
        double_t on_p_y;

        ActionID action_id;
    };

    class ActionCycleMoving
    {
    public:

        ActionCycleMoving();

        virtual ~ActionCycleMoving();

        ActionID getActionId() const;

        TargetVelXYYawPosZ calculateVelocity(double_t _target_altitude, double_t _target_radius, double_t _truth_radius);

    private:

        ActionID action_id;

        tf::TransformListener odom_base_tf_listener;

    };


    class ActionGoToLocalPositionHoldYaw
    {
    public:

        ActionGoToLocalPositionHoldYaw();

        virtual ~ActionGoToLocalPositionHoldYaw();

        ActionID getActionId() const;

        TargetPosXYZYaw calculateVelocity(geometry_msgs::Point _target_local_point);

        int8_t resetTargetYaw(double_t _new_target_yaw);

    private:

        ActionID action_id;
        double_t target_yaw;

        tf::TransformListener odom_base_tf_listener;

    };
}

#endif //FIRA_ESI_ACTION_H_
