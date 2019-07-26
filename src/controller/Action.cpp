//
// Created by kehan on 19-7-11.
//

#include "Action.h"
#include "PIDController.h"


vwpp::Action::Action()
{

}


vwpp::Action::~Action()
{
    delete instance;
}


vwpp::Action::Action(const vwpp::Action &)
{

}


vwpp::Action &vwpp::Action::operator=(const vwpp::Action &)
{

}


vwpp::Action* vwpp::Action::instance = nullptr;

boost::mutex vwpp::Action::mutex_instance;


vwpp::Action* vwpp::Action::getInstance()
{
    if (instance == nullptr)
    {
        boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
        if (instance == nullptr)
        {
            instance = new Action();
        }
    }

    return instance;
}


vwpp::Velocity2D
vwpp::Action::trackingLine(double_t _cur_line_y, double_t _target_yaw, double_t _cur_yaw,
                           double_t _forward_vel)
{

    static PIDController pid_controller_body_y(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKp(),
                                               vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKi(),
                                               vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKd());

    // TODO Change name 'body' to 'local'.
    static PIDController pid_controller_body_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawKp(),
                                                 vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawKi(),
                                                 vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawKd());


    pid_controller_body_y.setTarget(0.);
    pid_controller_body_y.update(_cur_line_y);

    pid_controller_body_yaw.setTarget(_target_yaw);
    pid_controller_body_yaw.update(convertCurYaw2FabsYawThetaBetweenPI(pid_controller_body_yaw.getTarget(),
                                                                       _cur_yaw));

    geometry_msgs::Vector3Stamped linear_body_vel{};
    linear_body_vel.header.stamp = ros::Time::now();
    linear_body_vel.header.frame_id = "camera_link";
    linear_body_vel.vector.x = _forward_vel;
    // TODO The direction of y vel maybe opposite.
    linear_body_vel.vector.y = pid_controller_body_y.output();
    // TODO Add velocity z.
    linear_body_vel.vector.z = 0;

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


    Velocity2D velocity_2d{};
    velocity_2d.x = linear_local_vel.vector.x;
    velocity_2d.y = linear_local_vel.vector.y;
    velocity_2d.yaw = pid_controller_body_yaw.output();

    return velocity_2d;
}


vwpp::Linear3D
vwpp::Action::adjustAltitude(double_t _target_altitude, double_t _cur_altitude, double_t _cur_x, double_t _cur_y)
{
    // TODO Could use the tf_transform, but because the z axis is always coincident, so...
    // TODO Use PX4 data to make the UAV stable. Maybe use vision.

    static PIDController pid_controller_local_x(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKp(),
                                                vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKi(),
                                                vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKd());

    static PIDController pid_controller_local_y(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKp(),
                                                vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKi(),
                                                vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKd());

    static PIDController pid_controller_local_z(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKp(),
                                                vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKi(),
                                                vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKd());

    pid_controller_local_x.setTarget(0.);
    pid_controller_local_y.setTarget(0.);
    pid_controller_local_z.setTarget(_target_altitude);

    pid_controller_local_x.update(_cur_x);
    pid_controller_local_y.update(_cur_y);
    pid_controller_local_z.update(_cur_altitude);

    Linear3D linear_3d{};
    linear_3d.x = pid_controller_local_x.output();
    linear_3d.y = pid_controller_local_y.output();
    linear_3d.z = pid_controller_local_z.output();

    return linear_3d;
}


vwpp::Velocity2D vwpp::Action::hovering(double_t _cur_x, double_t _cur_y)
{
    static PIDController pid_controller_body_x(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXKp(),
                                               vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXKi(),
                                               vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXKd());

    static PIDController pid_controller_body_y(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKp(),
                                               vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKi(),
                                               vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKd());

    static PIDController pid_controller_body_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawKp(),
                                                 vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawKi(),
                                                 vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYawKd());



    // Different task maybe different.
    pid_controller_body_x.setTarget(0.);
    pid_controller_body_x.update(_cur_x);
    pid_controller_body_y.setTarget(0.);
    pid_controller_body_y.update(_cur_y);

    // pid_controller_body_yaw.setTarget(_target_yaw);
    // pid_controller_body_yaw.update(_cur_yaw);

    geometry_msgs::Vector3Stamped linear_body_vel{};
    linear_body_vel.header.stamp = ros::Time::now();
    linear_body_vel.header.frame_id = "camera_link";
    // TODO The direction of x vel maybe opposite.
    linear_body_vel.vector.x = pid_controller_body_x.output();
    // TODO The direction of y vel maybe opposite.
    linear_body_vel.vector.y = pid_controller_body_y.output();
    linear_body_vel.vector.z = 0;

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

    Velocity2D velocity_2d{};
    velocity_2d.x = linear_local_vel.vector.x;
    velocity_2d.y = linear_local_vel.vector.y;
    // velocity_2d.yaw = pid_controller_body_yaw.output();
    // TODO
    velocity_2d.yaw = 0.;

    return velocity_2d;
}


vwpp::Velocity2D vwpp::Action::rotating(vwpp::Direction _direction, double_t _cur_yaw)
{
    double_t yaw_target = 0.;
    static PIDController pid_controller_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKp(),
                                            vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKi(),
                                            vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKd());

    switch (_direction)
    {
        case LOCAL_FORWARD:
            yaw_target = 0.;
            break;
        case LOCAL_LEFT:
            yaw_target = M_PI / 2;
            break;
        case LOCAL_BACK:
            yaw_target = M_PI;
            break;
        case LOCAL_RIGHT:
            yaw_target = M_PI * 1.5;
            break;
    }

    pid_controller_yaw.setTarget(yaw_target);
    pid_controller_yaw.update(convertCurYaw2FabsYawThetaBetweenPI(pid_controller_yaw.getTarget(),
                                                                  _cur_yaw));

    Velocity2D linear_local_vel{};
    linear_local_vel.x = 0.;
    linear_local_vel.y = 0.;
    linear_local_vel.yaw = pid_controller_yaw.output();

    return linear_local_vel;
}


vwpp::Velocity2D vwpp::Action::rotating(double_t _target_yaw, double_t _cur_yaw)
{

    static PIDController pid_controller_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKp(),
                                            vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKi(),
                                            vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKd());

    pid_controller_yaw.setTarget(_target_yaw);
    pid_controller_yaw.update(convertCurYaw2FabsYawThetaBetweenPI(pid_controller_yaw.getTarget(),
                                                                  _cur_yaw));

    Velocity2D linear_local_vel{};
    linear_local_vel.x = 0.;
    linear_local_vel.y = 0.;
    linear_local_vel.yaw = pid_controller_yaw.output();

    return linear_local_vel;
}


int8_t vwpp::Action::openClaw()
{
    return 0;
}

