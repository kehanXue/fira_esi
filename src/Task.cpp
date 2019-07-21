//
// Created by kehan on 19-7-11.
//

#include "Task.h"


// TODO
vwpp::TaskBase::TaskBase() :
        nh("~")
{

}


vwpp::TaskBase::TaskBase(vwpp::TaskID _task_id) :
        nh("~"),
        task_id(_task_id),
        task_state(SUSPEND)
{

}


vwpp::TaskBase::~TaskBase()
= default;


vwpp::TaskNavigation::TaskNavigation()
{
    p_task_base = new TaskBase(NAVIGATION);
    p_task_base->task_state = START;

    cur_action_id = TRACKINGLINE;

}


vwpp::TaskNavigation::~TaskNavigation()
{
    delete p_task_base;
}


vwpp::TaskID vwpp::TaskNavigation::getTaskID()
{
    return p_task_base->task_id;
}


vwpp::TaskState vwpp::TaskNavigation::getTaskState()
{
    return p_task_base->task_state;
}


int8_t vwpp::TaskNavigation::run()
{
    VisionInterface::getInstance()->update();
    PX4Interface::getInstance()->update();

    if (cur_action_id == TRACKINGLINE)
    {

        // TODO
        Velocity2D velocity_2d = Action::getInstance()->trackingLine(VisionInterface::getInstance()->getLineOffset(),
                                                                     PX4Interface::getInstance()->getCurYaw() +
                                                                     VisionInterface::getInstance()->getLineRotation(),
                                                                     PX4Interface::getInstance()->getCurYaw());


        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = "camera_odom_frame";
        cmd_vel.twist.linear.x = velocity_2d.x;
        cmd_vel.twist.linear.y = velocity_2d.y;
        cmd_vel.twist.linear.z = 0.;
        cmd_vel.twist.angular.z = velocity_2d.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }

    p_task_base->task_state = FINISH;
    return 0;
}


vwpp::TaskAvoidance::TaskAvoidance()
{
    p_task_base = new TaskBase(AVOIDANCE);

    cur_action_id = ADJUSTALTITUDE;

    altitude_target = 0.0;

    forward_counter = 0;
}


vwpp::TaskAvoidance::~TaskAvoidance()
{
    delete p_task_base;
}


vwpp::TaskID vwpp::TaskAvoidance::getTaskID()
{
    return p_task_base->task_id;
}


vwpp::TaskState vwpp::TaskAvoidance::getTaskState()
{
    return p_task_base->task_state;
}


int8_t vwpp::TaskAvoidance::run(GateType _gate_type)
{

    VisionInterface::getInstance()->update();
    PX4Interface::getInstance()->update();

    static int inter_adjust_altitude_time = 1;

    if (cur_action_id == ADJUSTALTITUDE)
    {

        VelocityZ velocity_z = 0;

        if (inter_adjust_altitude_time == 1)
        {
            if (_gate_type == RED)
            {
                // TODO Tuning param
                this->altitude_target = 1.5;
            }
            else if (_gate_type == YELLOW)
            {
                this->altitude_target = 0.5;
            }

            velocity_z = Action::getInstance()->adjustAltitude(this->altitude_target,
                                                               PX4Interface::getInstance()->getCurZ());

            // TODO param
            if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <= 0.10)
            {
                cur_action_id = TRACKINGLINE;
            }

        }
        else if (inter_adjust_altitude_time == 2)
        {
            this->altitude_target = 1.0;

            velocity_z = Action::getInstance()->adjustAltitude(this->altitude_target,
                                                               PX4Interface::getInstance()->getCurZ());

            // TODO param
            if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <= 0.10)
            {
                inter_adjust_altitude_time = 1;
                p_task_base->task_state = FINISH;
                return 1;
            }

        }

        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = "camera_odom_frame";
        cmd_vel.twist.linear.x = 0.;
        cmd_vel.twist.linear.y = 0.;
        cmd_vel.twist.linear.z = velocity_z;
        cmd_vel.twist.angular.z = 0.;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);

    }
    else if (cur_action_id == TRACKINGLINE)
    {
        forward_counter++;
        // TODO param
        if (forward_counter >= 10)
        {
            cur_action_id = ADJUSTALTITUDE;
            inter_adjust_altitude_time += 1;
        }

        Velocity2D velocity_2d = Action::getInstance()->trackingLine(VisionInterface::getInstance()->getLineOffset(),
                                                                     PX4Interface::getInstance()->getCurYaw() +
                                                                     VisionInterface::getInstance()->getLineRotation(),
                                                                     PX4Interface::getInstance()->getCurYaw());

        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = "camera_odom_frame";
        cmd_vel.twist.linear.x = velocity_2d.x;
        cmd_vel.twist.linear.y = velocity_2d.y;
        cmd_vel.twist.linear.z = 0.;
        cmd_vel.twist.angular.z = velocity_2d.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }

    p_task_base->task_state = PROCESSING;
    return 0;

}


vwpp::TaskHoverOnQR::TaskHoverOnQR()
{
    p_task_base = new TaskBase(HOVERONQR);

    cur_action_id = HOVERING;

}


vwpp::TaskHoverOnQR::~TaskHoverOnQR()
{
    delete p_task_base;
}


vwpp::TaskID vwpp::TaskHoverOnQR::getTaskID()
{
    return p_task_base->task_id;
}


vwpp::TaskState vwpp::TaskHoverOnQR::getTaskState()
{
    return p_task_base->task_state;
}


vwpp::CurrentQRTaskID vwpp::TaskHoverOnQR::run(TaskID _cur_task_id)
{

    VisionInterface::getInstance()->update();
    PX4Interface::getInstance()->update();

    static int inter_adjust_altitude_time = 1;

    if (cur_action_id == HOVERING)
    {
        Velocity2D velocity_2d = Action::getInstance()->hovering(VisionInterface::getInstance()->getQRxOffset(),
                                                                 VisionInterface::getInstance()->getQRyOffset());

        if (inter_adjust_altitude_time == 2)
        {
            p_task_base->task_state = FINISH;
        }

        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = "camera_odom_frame";
        cmd_vel.twist.linear.x = velocity_2d.x;
        cmd_vel.twist.linear.y = velocity_2d.y;
        cmd_vel.twist.linear.z = 0.;
        cmd_vel.twist.angular.z = velocity_2d.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }
    else if (cur_action_id == ROTATION)
    {
        std::string qr_inform = VisionInterface::getInstance()->getGroundQRinform();

        // TODO
        Direction target_direction = LOCAL_FORWARD;         // Init
        switch (qr_inform.at(_cur_task_id))
        {
            case 'N':
                target_direction = LOCAL_FORWARD;
                break;
            case 'W':
                target_direction = LOCAL_LEFT;
                break;
            case 'S':
                target_direction = LOCAL_BACK;
                break;
            case 'E':
                target_direction = LOCAL_RIGHT;
                break;
        }

        // TODO Add a & param


        Velocity2D velocity_2d = Action::getInstance()->rotating(target_direction,
                                                                 PX4Interface::getInstance()->getCurYaw());
        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = "camera_odom_frame";
        cmd_vel.twist.linear.x = velocity_2d.x;
        cmd_vel.twist.linear.y = velocity_2d.y;
        cmd_vel.twist.linear.z = 0.;
        cmd_vel.twist.angular.z = velocity_2d.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }



    p_task_base->task_state = PROCESSING;
    return 0;
}


