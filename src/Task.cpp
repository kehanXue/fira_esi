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

    p_task_base->task_state = START;

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
                p_task_base->task_state = FINISH;
                inter_adjust_altitude_time = 1;
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

    p_task_base->task_state = START;
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


char vwpp::TaskHoverOnQR::run(TaskID _cur_task_id)
{

    VisionInterface::getInstance()->update();
    PX4Interface::getInstance()->update();
    std::string qr_inform = VisionInterface::getInstance()->getGroundQRinform();

    static int inter_hovering_time = 1;

    if (cur_action_id == HOVERING)
    {
        Velocity2D velocity_2d = Action::getInstance()->hovering(VisionInterface::getInstance()->getQRxOffset(),
                                                                 VisionInterface::getInstance()->getQRyOffset());

        if (inter_hovering_time == 2)
        {
            p_task_base->task_state = FINISH;
            inter_hovering_time = 1;

            return qr_inform.at(qr_inform.size() - 1);
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
        double_t yaw_target = 0;
        switch (target_direction)
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
        // TODO
        if (fabs(yaw_target - PX4Interface::getInstance()->getCurYaw()) <= 3.)
        {
            cur_action_id = HOVERING;
            inter_hovering_time += 1;
        }
        else
        {
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

    }


    p_task_base->task_state = PROCESSING;
    return qr_inform.at(qr_inform.size() - 1);


    // switch (qr_inform.at(qr_inform.size()-1))
    // {
    //     case '1':
    //         current_qr_task_id = DELIVERING;
    //         break;
    //     case '2':
    //         current_qr_task_id = SCANTOWER;
    //         break;
    //     case '3':
    //         current_qr_task_id = SCANBUILDING;
    //         break;
    //     case '4':
    //         current_qr_task_id = LANDING;
    //         break;
    // }

}


vwpp::TaskDelivering::TaskDelivering()
{

    p_task_base = new TaskBase(DELIVERING);

    p_task_base->task_state = START;

    cur_action_id = TRACKINGLINE;

}


vwpp::TaskDelivering::~TaskDelivering()
{
    delete p_task_base;
}


vwpp::TaskID vwpp::TaskDelivering::getTaskID()
{
    return p_task_base->task_id;
}


vwpp::TaskState vwpp::TaskDelivering::getTaskState()
{
    return p_task_base->task_state;
}


int8_t vwpp::TaskDelivering::run()
{

    double_t back_toward_yaw = 0;
    VisionInterface::getInstance()->update();
    PX4Interface::getInstance()->update();

    if (cur_action_id == TRACKINGLINE)
    {
        if (VisionInterface::getInstance()->getRedXState())
        {
            cur_action_id = HOVERING;
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
    else if (cur_action_id == HOVERING)
    {
        // TODO param
        if (fabs(VisionInterface::getInstance()->getRedXx() - 0) <= 10 &&
            fabs(VisionInterface::getInstance()->getRedXy() - 0) <= 10)
        {
            cur_action_id = OPENCLAW;
        }

        Velocity2D velocity_2d = Action::getInstance()->hovering(VisionInterface::getInstance()->getRedXx(),
                                                                 VisionInterface::getInstance()->getRedXy());

        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = "camera_odom_frame";
        cmd_vel.twist.linear.x = velocity_2d.x;
        cmd_vel.twist.linear.y = velocity_2d.y;
        cmd_vel.twist.linear.z = 0.;
        cmd_vel.twist.angular.z = velocity_2d.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }
    else if (cur_action_id == OPENCLAW)
    {
        // TODO
        Action::getInstance()->openClaw();
        cur_action_id = ROTATION;
        // TODO
        back_toward_yaw = (PX4Interface::getInstance()->getCurYaw() + 180) / 360.0;
    }
    else if (cur_action_id == ROTATION)
    {
        // TODO
        if (fabs(PX4Interface::getInstance()->getCurYaw() - back_toward_yaw) <= 5)
        {
            p_task_base->task_state = FINISH;
            return 1;
        }

        Velocity2D velocity_2d = Action::getInstance()->rotating(back_toward_yaw,
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


vwpp::TaskLanding::TaskLanding()
{
    p_task_base = new TaskBase(LANDING);

    cur_action_id = TRACKINGLINE;

    p_task_base->task_state = START;
}


vwpp::TaskLanding::~TaskLanding()
{
    delete p_task_base;
}


vwpp::TaskID vwpp::TaskLanding::getTaskID()
{
    return p_task_base->task_id;
}


vwpp::TaskState vwpp::TaskLanding::getTaskState()
{
    return p_task_base->task_state;
}


int8_t vwpp::TaskLanding::run()
{
    VisionInterface::getInstance()->update();
    PX4Interface::getInstance()->update();

    if (cur_action_id == TRACKINGLINE)
    {
        if (VisionInterface::getInstance()->getBlueHState())
        {
            cur_action_id = HOVERING;
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
    else if (cur_action_id == HOVERING)
    {
        // TODO param
        if (fabs(VisionInterface::getInstance()->getBlueHx() - 0) <= 10 &&
            fabs(VisionInterface::getInstance()->getBlueHy() - 0) <= 10)
        {
            cur_action_id = ADJUSTALTITUDE;
        }

        Velocity2D velocity_2d = Action::getInstance()->hovering(VisionInterface::getInstance()->getBlueHx(),
                                                                 VisionInterface::getInstance()->getBlueHy());

        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = "camera_odom_frame";
        cmd_vel.twist.linear.x = velocity_2d.x;
        cmd_vel.twist.linear.y = velocity_2d.y;
        cmd_vel.twist.linear.z = 0.;
        cmd_vel.twist.angular.z = velocity_2d.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }
    else if (cur_action_id == ADJUSTALTITUDE)
    {
        // TODO param
        double_t altitude_target = 0.;

        // TODO param
        if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <= 0.05)
        {
            p_task_base->task_state = FINISH;
            return 1;
        }


        VelocityZ velocity_z = Action::getInstance()->adjustAltitude(altitude_target,
                                                                     PX4Interface::getInstance()->getCurZ());

        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = "camera_odom_frame";
        cmd_vel.twist.linear.x = 0.;
        cmd_vel.twist.linear.y = 0.;
        cmd_vel.twist.linear.z = velocity_z;
        cmd_vel.twist.angular.z = 0.;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);

    }

    p_task_base->task_state = PROCESSING;
    return 0;
}

