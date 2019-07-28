//
// Created by kehan on 19-7-27.
//

#include "Action.h"


ActionBase::ActionBase() :
        action_id(TRACKINGLINE)
{

}


ActionBase::~ActionBase()
= default;


ActionTrackingLine::ActionTrackingLine(double_t _target_altitude) :
        action_id(TRACKINGLINE),
        target_altitude(_target_altitude)
{

}


ActionTrackingLine::~ActionTrackingLine()
= default;


ActionID ActionTrackingLine::getActionID()
{
    return action_id;
}


DroneVelocity ActionTrackingLine::calculateVelocity(double_t _cur_line_v_y, double_t _cur_v_yaw, double_t _forward_vel)
{
    // v means data from vision, p means data from px4.

    static vwpp::PIDController
            pid_controller_v_body_y(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKp(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKi(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKd(),
                                    vwpp::DynamicRecfgInterface::getInstance()->isPidVV2PYHasThreshold(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYThreshold());

    static vwpp::PIDController
            pid_controller_p_local_z(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PZHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZThreshold());

    static vwpp::PIDController
            pid_controller_v_body_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawKp(),
                                      vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawKi(),
                                      vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawKd(),
                                      vwpp::DynamicRecfgInterface::getInstance()->isPidVV2PYawHasThreshold(),
                                      vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawThreshold());


    pid_controller_v_body_y.setTarget(0.);
    pid_controller_p_local_z.setTarget(this->target_altitude);
    pid_controller_v_body_yaw.setTarget(0.);

    pid_controller_v_body_y.update(_cur_line_v_y);
    pid_controller_p_local_z.update(vwpp::PX4Interface::getInstance()->getCurZ());
    pid_controller_v_body_yaw.update(
            convertCurYaw2FabsYawThetaBetweenPI(pid_controller_v_body_yaw.getTarget(), _cur_v_yaw));


    geometry_msgs::Vector3Stamped linear_body_vel{};
    linear_body_vel.header.stamp = ros::Time::now();
    // TODO param
    linear_body_vel.header.frame_id = "camera_link";
    linear_body_vel.vector.x = _forward_vel;
    linear_body_vel.vector.y = pid_controller_v_body_y.output();
    // TODO Test z
    linear_body_vel.vector.z = pid_controller_p_local_z.output();

    geometry_msgs::Vector3Stamped linear_local_vel{};
    try
    {
        odom_base_tf_listener.transformVector("camera_odom_frame", linear_body_vel, linear_local_vel);
    }
    catch (tf::TransformException &tf_ex)
    {
        ROS_ERROR("%s", tf_ex.what());
        ros::Duration(vwpp::DynamicRecfgInterface::getInstance()->getTfBreakDuration()).sleep();
    }

    DroneVelocity drone_velocity{};
    drone_velocity.x = linear_local_vel.vector.x;
    drone_velocity.y = linear_local_vel.vector.y;
    drone_velocity.z = linear_local_vel.vector.z;
    drone_velocity.yaw = pid_controller_v_body_yaw.output();

    return drone_velocity;
}


ActionHovering::ActionHovering(double_t _target_altitude) :
        action_id(HOVERING),
        target_altitude(_target_altitude)
{
    target_yaw = vwpp::PX4Interface::getInstance()->getCurYaw();

}


ActionHovering::~ActionHovering()
= default;


ActionID ActionHovering::getActionId() const
{
    return action_id;
}


DroneVelocity ActionHovering::calculateVelocity(double_t _cur_v_x, double_t _cur_v_y)
{

    static vwpp::PIDController
            pid_controller_v_body_x(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXKp(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXKi(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXKd(),
                                    vwpp::DynamicRecfgInterface::getInstance()->isPidVV2PXHasThreshold(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXThreshold());

    static vwpp::PIDController
            pid_controller_v_body_y(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKp(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKi(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKd(),
                                    vwpp::DynamicRecfgInterface::getInstance()->isPidVV2PYHasThreshold(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYThreshold());
    static vwpp::PIDController
            pid_controller_p_local_z(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PZHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZThreshold());

    static vwpp::PIDController
            pid_controller_p_local_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKp(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKi(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKd(),
                                       vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYawHasThreshold(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawThreshold());


    pid_controller_v_body_x.setTarget(0.);
    pid_controller_v_body_y.setTarget(0.);
    pid_controller_p_local_z.setTarget(this->target_altitude);
    // TODO
    pid_controller_p_local_yaw.setTarget(this->target_yaw);


    pid_controller_v_body_x.update(_cur_v_x);
    pid_controller_v_body_y.update(_cur_v_y);
    pid_controller_p_local_z.update(vwpp::PX4Interface::getInstance()->getCurZ());
    pid_controller_p_local_yaw.update(
            convertCurYaw2FabsYawThetaBetweenPI(target_yaw, vwpp::PX4Interface::getInstance()->getCurZ()));

    geometry_msgs::Vector3Stamped linear_body_vel{};
    linear_body_vel.header.stamp = ros::Time::now();
    linear_body_vel.header.frame_id = "camera_link";
    linear_body_vel.vector.x = pid_controller_v_body_x.output();
    linear_body_vel.vector.y = pid_controller_v_body_y.output();
    linear_body_vel.vector.z = pid_controller_p_local_z.output();

    geometry_msgs::Vector3Stamped linear_local_vel{};
    try
    {
        odom_base_tf_listener.transformVector("camera_odom_frame", linear_body_vel, linear_local_vel);
    }
    catch (tf::TransformException &tf_ex)
    {
        ROS_ERROR("%s", tf_ex.what());
        ros::Duration(vwpp::DynamicRecfgInterface::getInstance()->getTfBreakDuration()).sleep();
    }

    DroneVelocity drone_velocity{};
    drone_velocity.x = linear_local_vel.vector.x;
    drone_velocity.y = linear_local_vel.vector.y;
    drone_velocity.z = linear_local_vel.vector.z;
    drone_velocity.yaw = pid_controller_p_local_yaw.output();


    return drone_velocity;
}


ActionRotating::ActionRotating(double_t _target_altitude) :
        action_id(ROTATION),
        target_altitude(_target_altitude)
{
    initial_p_x = vwpp::PX4Interface::getInstance()->getCurX();
    initial_p_y = vwpp::PX4Interface::getInstance()->getCurY();
}


ActionRotating::~ActionRotating()
= default;


ActionID ActionRotating::getActionId() const
{
    return action_id;
}


DroneVelocity ActionRotating::calculateVelocity(double_t _target_yaw, double_t _cur_yaw)
{

    static vwpp::PIDController
            pid_controller_p_local_x(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PXHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXThreshold());

    static vwpp::PIDController
            pid_controller_p_local_y(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYThreshold());
    static vwpp::PIDController
            pid_controller_p_local_z(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PZHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZThreshold());

    static vwpp::PIDController
            pid_controller_p_local_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKp(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKi(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKd(),
                                       vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYawHasThreshold(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawThreshold());

    // TODO Local->Body?
    pid_controller_p_local_x.setTarget(this->initial_p_x);
    pid_controller_p_local_y.setTarget(this->initial_p_y);
    pid_controller_p_local_z.setTarget(this->target_altitude);
    pid_controller_p_local_yaw.setTarget(_target_yaw);

    pid_controller_p_local_x.update(vwpp::PX4Interface::getInstance()->getCurX());
    pid_controller_p_local_y.update(vwpp::PX4Interface::getInstance()->getCurY());
    pid_controller_p_local_z.update(vwpp::PX4Interface::getInstance()->getCurZ());
    pid_controller_p_local_yaw.update(convertCurYaw2FabsYawThetaBetweenPI(_target_yaw, _cur_yaw));

    DroneVelocity drone_velocity{};
    drone_velocity.x = pid_controller_p_local_x.output();
    drone_velocity.y = pid_controller_p_local_y.output();
    drone_velocity.z = pid_controller_p_local_z.output();
    drone_velocity.yaw = pid_controller_p_local_yaw.output();

    return drone_velocity;
}


ActionAdjustAltitude::ActionAdjustAltitude() :
        action_id(ADJUSTALTITUDE)
{
    initial_p_x = vwpp::PX4Interface::getInstance()->getCurX();
    initial_p_y = vwpp::PX4Interface::getInstance()->getCurY();
    initial_p_yaw = vwpp::PX4Interface::getInstance()->getCurYaw();
}


ActionAdjustAltitude::~ActionAdjustAltitude()
= default;


ActionID ActionAdjustAltitude::getActionId() const
{
    return action_id;
}


DroneVelocity ActionAdjustAltitude::calculateVelocity(double_t _target_altitude, double_t _cur_altitude)
{
    static vwpp::PIDController
            pid_controller_p_local_x(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PXHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXThreshold());

    static vwpp::PIDController
            pid_controller_p_local_y(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYThreshold());

    static vwpp::PIDController
            pid_controller_p_local_z(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PZHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZThreshold());

    static vwpp::PIDController
            pid_controller_p_local_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKp(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKi(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKd(),
                                       vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYawHasThreshold(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawThreshold());

    pid_controller_p_local_x.setTarget(this->initial_p_x);
    pid_controller_p_local_y.setTarget(this->initial_p_y);
    pid_controller_p_local_z.setTarget(_target_altitude);
    pid_controller_p_local_yaw.setTarget(this->initial_p_yaw);

    pid_controller_p_local_x.update(vwpp::PX4Interface::getInstance()->getCurX());
    pid_controller_p_local_y.update(vwpp::PX4Interface::getInstance()->getCurY());
    pid_controller_p_local_z.update(vwpp::PX4Interface::getInstance()->getCurZ());
    pid_controller_p_local_yaw.update(
            convertCurYaw2FabsYawThetaBetweenPI(initial_p_yaw, vwpp::PX4Interface::getInstance()->getCurYaw()));

    DroneVelocity drone_velocity{};
    drone_velocity.x = pid_controller_p_local_x.output();
    drone_velocity.y = pid_controller_p_local_y.output();
    drone_velocity.z = pid_controller_p_local_z.output();
    drone_velocity.yaw = pid_controller_p_local_yaw.output();

    return drone_velocity;
}


