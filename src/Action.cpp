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
    // TODO Get param from ros param server.
    static PIDController pid_controller_body_y(1.0, 0.0, 1.0);
    static PIDController pid_controller_body_yaw(1.0, 0.0, 1.0);

    pid_controller_body_y.setTarget(0);
    pid_controller_body_y.update(_cur_line_y);

    pid_controller_body_yaw.setTarget(_target_yaw);
    pid_controller_body_yaw.update(_cur_yaw);

    geometry_msgs::Vector3Stamped linear_body_vel{};
    linear_body_vel.header.stamp = ros::Time::now();
    linear_body_vel.header.frame_id = "camera_link";
    linear_body_vel.vector.x = _forward_vel;
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
        ros::Duration(1.0).sleep();
    }


    Velocity2D velocity_2d{};
    velocity_2d.x = linear_local_vel.vector.x;
    velocity_2d.y = linear_local_vel.vector.y;
    velocity_2d.yaw = pid_controller_body_yaw.output();

    return velocity_2d;
}


vwpp::VelocityZ vwpp::Action::adjustAltitude(double_t _target_altitude, double_t _cur_altitude)
{
    // TODO Could use the tf_transform, but because the z axis is always coincident, so...
    static PIDController pid_controller_local_z(1.0, 0.0, 1.0);
    pid_controller_local_z.setTarget(_target_altitude);
    pid_controller_local_z.update(_cur_altitude);

    return pid_controller_local_z.output();
}


vwpp::Velocity2D vwpp::Action::hovering(double_t _cur_x, double_t _cur_y, double_t _target_yaw, double_t _cur_yaw)
{
    // TODO Get param from ros param server.
    static PIDController pid_controller_body_x(1.0, 0.0, 1.0);
    static PIDController pid_controller_body_y(1.0, 0.0, 1.0);
    static PIDController pid_controller_body_yaw(1.0, 0.0, 1.0);

    pid_controller_body_x.setTarget(0);
    pid_controller_body_x.update(_cur_x);
    pid_controller_body_y.setTarget(0);
    pid_controller_body_y.update(_cur_y);

    pid_controller_body_yaw.setTarget(_target_yaw);
    pid_controller_body_yaw.update(_cur_yaw);

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
        ros::Duration(1.0).sleep();
    }

    Velocity2D velocity_2d{};
    velocity_2d.x = linear_local_vel.vector.x;
    velocity_2d.y = linear_local_vel.vector.y;
    velocity_2d.yaw = pid_controller_body_yaw.output();

    return velocity_2d;
}


vwpp::Velocity2D vwpp::Action::rotating(vwpp::Direction _direction, double_t _cur_yaw)
{
    double_t yaw_target = 0.;
    // TODO Need to be turning.
    static PIDController pid_controller_yaw(1.0, 0, 1.0);

    switch (_direction)
    {
        case LOCAL_FORWARD:
            yaw_target = 0.;
            break;
        case LOCAL_LEFT:
            yaw_target = 90.;
            break;
        case LOCAL_BACK:
            yaw_target = 180.;
            break;
        case LOCAL_RIGHT:
            yaw_target = 270.;
            break;
    }

    pid_controller_yaw.setTarget(yaw_target);
    pid_controller_yaw.update(_cur_yaw);

    Velocity2D linear_local_vel{};
    linear_local_vel.x = 0;
    linear_local_vel.y = 0;
    linear_local_vel.yaw = pid_controller_yaw.output();

    return linear_local_vel;
}


int8_t vwpp::Action::openClaw()
{
    return 0;
}


// int8_t vwpp::Action::run(vwpp::ActionID _action_id)
// {
//
//    Velocity2D velocity_2d{};
//    VelocityZ velocity_z{};
//
//    switch (_action_id)
//    {
//        case TRACKINGLINE:
//            velocity_2d = trackingLine(getLineCurX(), getTargetYaw(), getCurYaw(),
//                                       1.0);     // TODO Modify vision api, and get param from yaml.
//            break;
//        case ADJUSTALTITUDE:
//            velocity_z = adjustAltitude(getTargetAltitude(), getCurAltitude());
//            break;
//        case HOVERING:
//            velocity_2d = hovering(getCurQRx(), getCurQRy(), getTargetYaw(), getCurYaw());
//            break;
//        case ROTATION:
//            velocity_2d = rotating(getDirection(), getCurYaw());
//            break;
//        case OPENCLAW:
//            openClaw();
//            break;
//        case CIRCULARMOTION:
//            break;
//    }
//
//    return 0;
// }











