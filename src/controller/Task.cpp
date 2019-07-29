//
// Created by kehan on 19-7-11.
//

#include "Task.h"


// TODO
vwpp::TaskBase::TaskBase() :
        nh("~"),
        task_id(NAVIGATION),
        task_state(TASK_SUSPEND)
{

}


vwpp::TaskBase::TaskBase(vwpp::TaskID _task_id) :
        nh("~"),
        task_id(_task_id),
        task_state(TASK_SUSPEND)
{

}


vwpp::TaskBase::~TaskBase()
= default;


vwpp::TaskNavigation::TaskNavigation()
{
    p_task_base = new TaskBase(NAVIGATION);
    p_task_base->task_state = TASK_START;

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
    // VisionInterface::getInstance()->update();
    // PX4Interface::getInstance()->update();

    if (cur_action_id == TRACKINGLINE)
    {

        static ActionTrackingLine action_tracking_line(
                vwpp::DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());

        DroneVelocity drone_velocity =
                action_tracking_line.calculateVelocity(
                        vwpp::VisionInterface::getInstance()->getLineOffset(),
                        vwpp::VisionInterface::getInstance()->getLineRotation());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }

    p_task_base->task_state = TASK_FINISH;
    return 0;
}


vwpp::TaskAvoidance::TaskAvoidance()
{
    p_task_base = new TaskBase(AVOIDANCE);

    cur_action_id = ADJUSTALTITUDE;

    p_task_base->task_state = TASK_START;

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

    // VisionInterface::getInstance()->update();
    // PX4Interface::getInstance()->update();

    geometry_msgs::Twist cmd_vel;
    static int inter_adjust_altitude_time = 1;

    if (cur_action_id == ADJUSTALTITUDE)
    {

        if (inter_adjust_altitude_time == 1)
        {

            if (_gate_type == RED)
            {
                this->altitude_target =
                        vwpp::DynamicRecfgInterface::getInstance()->getAltitudeWhenRedGate();
            }
            else if (_gate_type == YELLOW)
            {
                this->altitude_target =
                        vwpp::DynamicRecfgInterface::getInstance()->getAltitudeWhenYellowGate();
            }


            // TODO ptr
            static ActionAdjustAltitude action_adjust_altitude;
            DroneVelocity drone_velocity = action_adjust_altitude.calculateVelocity(this->altitude_target,
                                                                                    PX4Interface::getInstance()->getCurZ());

            cmd_vel.linear.x = drone_velocity.x;
            cmd_vel.linear.y = drone_velocity.y;
            cmd_vel.linear.z = drone_velocity.z;
            cmd_vel.angular.z = drone_velocity.yaw;


            if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <=
                vwpp::DynamicRecfgInterface::getInstance()->getAltitudeToleranceError())
            {
                static JudgeAchieveCounter judge_achieve_counter(
                        DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
                if (judge_achieve_counter.isAchieve())
                {
                    cur_action_id = TRACKINGLINE;
                }
            }

        }
        else if (inter_adjust_altitude_time == 2)
        {
            this->altitude_target =
                    vwpp::DynamicRecfgInterface::getInstance()->getNormalFlightAltitude();


            static ActionAdjustAltitude action_adjust_altitude;
            DroneVelocity drone_velocity = action_adjust_altitude.calculateVelocity(this->altitude_target,
                                                                                    PX4Interface::getInstance()->getCurZ());

            cmd_vel.linear.x = drone_velocity.x;
            cmd_vel.linear.y = drone_velocity.y;
            cmd_vel.linear.z = drone_velocity.z;
            cmd_vel.angular.z = drone_velocity.yaw;


            if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <=
                vwpp::DynamicRecfgInterface::getInstance()->getAltitudeToleranceError())
            {
                static JudgeAchieveCounter judge_achieve_counter(
                        DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
                if (judge_achieve_counter.isAchieve())
                {

                    p_task_base->task_state = TASK_FINISH;
                    inter_adjust_altitude_time = 1;
                    return 1;
                }
            }

        }

    }
    else if (cur_action_id == TRACKINGLINE)
    {
        forward_counter++;

        // 10: Controller Hz
        if (forward_counter >=
            10 * vwpp::DynamicRecfgInterface::getInstance()->getAvoidanceForwardTime())
        {
            cur_action_id = ADJUSTALTITUDE;
            inter_adjust_altitude_time += 1;
        }

        static ActionTrackingLine action_tracking_line(this->altitude_target);
        DroneVelocity drone_velocity =
                action_tracking_line.calculateVelocity(
                        vwpp::VisionInterface::getInstance()->getLineOffset(),
                        vwpp::VisionInterface::getInstance()->getLineRotation());


        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

    }

    PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    p_task_base->task_state = TASK_PROCESSING;
    return 0;

}


vwpp::TaskHoverOnQR::TaskHoverOnQR()
{
    p_task_base = new TaskBase(HOVERONQR);

    cur_action_id = HOVERING;

    p_task_base->task_state = TASK_START;
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

    // VisionInterface::getInstance()->update();
    // PX4Interface::getInstance()->update();
    std::string qr_inform = VisionInterface::getInstance()->getGroundQRinform();

    static int inter_hovering_time = 1;

    if (cur_action_id == HOVERING)
    {
        if (fabs(VisionInterface::getInstance()->getQRxOffset()) <
            vwpp::DynamicRecfgInterface::getInstance()->getQrOffsetXTolerance() &&
            fabs(VisionInterface::getInstance()->getQRyOffset()) <
            vwpp::DynamicRecfgInterface::getInstance()->getQrOffsetYTolerance())
        {
            if (inter_hovering_time == 1)
            {
                static JudgeAchieveCounter judge_achieve_counter(
                        DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
                if (judge_achieve_counter.isAchieve())
                {
                    cur_action_id = ROTATION;
                }
            }
            if (inter_hovering_time == 2)
            {
                static JudgeAchieveCounter judge_achieve_counter(
                        DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
                if (judge_achieve_counter.isAchieve())
                {
                    p_task_base->task_state = TASK_FINISH;
                    inter_hovering_time = 1;

                    return qr_inform.at(qr_inform.size() - 1);
                }
            }
        }

        static ActionHovering action_hovering(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());

        DroneVelocity drone_velocity =
                action_hovering.calculateVelocity(VisionInterface::getInstance()->getQRxOffset(),
                                                  VisionInterface::getInstance()->getQRyOffset());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

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
                yaw_target = M_PI / 2.;
                break;
            case LOCAL_BACK:
                yaw_target = M_PI;
                break;
            case LOCAL_RIGHT:
                yaw_target = M_PI * 1.5;
                break;
        }

        if (fabs(yaw_target - PX4Interface::getInstance()->getCurYaw()) <=
            vwpp::DynamicRecfgInterface::getInstance()->getRotateYawTolerance() * M_PI / 180.)
        {
            static JudgeAchieveCounter judge_achieve_counter(
                    DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
            if (judge_achieve_counter.isAchieve())
            {
                cur_action_id = HOVERING;
                inter_hovering_time += 1;
            }
        }

        else
        {
            static ActionRotating action_rotating(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());

            DroneVelocity drone_velocity = action_rotating.calculateVelocity(yaw_target,
                                                                             PX4Interface::getInstance()->getCurYaw());

            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = drone_velocity.x;
            cmd_vel.linear.y = drone_velocity.y;
            cmd_vel.linear.z = drone_velocity.z;
            cmd_vel.angular.z = drone_velocity.yaw;

            PX4Interface::getInstance()->publishLocalVel(cmd_vel);
        }

    }

    p_task_base->task_state = TASK_PROCESSING;
    return qr_inform.at(qr_inform.size() - 1);
}


vwpp::TaskDelivering::TaskDelivering()
{

    p_task_base = new TaskBase(DELIVERING);

    p_task_base->task_state = TASK_START;

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

    static double_t back_toward_yaw = 0;
    // VisionInterface::getInstance()->update();
    // PX4Interface::getInstance()->update();

    if (cur_action_id == TRACKINGLINE)
    {
        if (VisionInterface::getInstance()->getRedXState())
        {
            // TODO Who Judge?
            cur_action_id = HOVERING;
        }

        static ActionTrackingLine action_tracking_line(
                DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());

        DroneVelocity drone_velocity =
                action_tracking_line.calculateVelocity(
                        vwpp::VisionInterface::getInstance()->getLineOffset(),
                        vwpp::VisionInterface::getInstance()->getLineRotation());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }
    else if (cur_action_id == HOVERING)
    {
        // TODO param
        if (fabs(VisionInterface::getInstance()->getRedXx() - 0.) <=
            vwpp::DynamicRecfgInterface::getInstance()->getRedXOffsetXTolerance() &&
            fabs(VisionInterface::getInstance()->getRedXy() - 0.) <=
            vwpp::DynamicRecfgInterface::getInstance()->getRedXOffsetYTolerance())
        {
            static JudgeAchieveCounter judge_achieve_counter(
                    DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
            if (judge_achieve_counter.isAchieve())
            {
                cur_action_id = OPENCLAW;
            }
        }

        static ActionHovering action_hovering(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
        DroneVelocity drone_velocity =
                action_hovering.calculateVelocity(VisionInterface::getInstance()->getRedXx(),
                                                  VisionInterface::getInstance()->getRedXy());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }
    else if (cur_action_id == OPENCLAW)
    {
        // TODO
        // Action::getInstance()->openClaw();

        cur_action_id = ROTATION;

        // TODO Yaw Control.
        back_toward_yaw = (PX4Interface::getInstance()->getCurYaw() + M_PI);
    }
    else if (cur_action_id == ROTATION)
    {

        // TODO Maybe problems
        if (fabs(PX4Interface::getInstance()->getCurYaw() - back_toward_yaw) <=
            vwpp::DynamicRecfgInterface::getInstance()->getRotateYawTolerance() * M_PI / 180.)
        {
            static JudgeAchieveCounter judge_achieve_counter(
                    DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
            if (judge_achieve_counter.isAchieve())
            {
                p_task_base->task_state = TASK_FINISH;
                return 1;
            }
        }

        static ActionRotating action_rotating(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
        DroneVelocity drone_velocity =
                action_rotating.calculateVelocity(back_toward_yaw,
                                                  PX4Interface::getInstance()->getCurYaw());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }

    p_task_base->task_state = TASK_PROCESSING;
    return 0;
}


vwpp::TaskLanding::TaskLanding()
{
    p_task_base = new TaskBase(LANDING);

    // cur_action_id = TRACKINGLINE;
    cur_action_id = ADJUSTALTITUDE;

    p_task_base->task_state = TASK_START;
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
    // VisionInterface::getInstance()->update();
    // PX4Interface::getInstance()->update();

    if (cur_action_id == TRACKINGLINE)
    {
        if (VisionInterface::getInstance()->getBlueHState())
        {
            cur_action_id = HOVERING;
        }

        static ActionTrackingLine action_tracking_line(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());

        DroneVelocity drone_velocity =
                action_tracking_line.calculateVelocity(VisionInterface::getInstance()->getLineOffset(),
                                                       VisionInterface::getInstance()->getLineRotation());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }
    else if (cur_action_id == HOVERING)
    {
        if (fabs(VisionInterface::getInstance()->getBlueHx() - 0.) <=
            vwpp::DynamicRecfgInterface::getInstance()->getBlueHOffsetXTolerance() &&
            fabs(VisionInterface::getInstance()->getBlueHy() - 0.) <=
            vwpp::DynamicRecfgInterface::getInstance()->getBlueHOffsetYTolerance())
        {

            static JudgeAchieveCounter judge_achieve_counter(
                    DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
            if (judge_achieve_counter.isAchieve())
            {
                cur_action_id = ADJUSTALTITUDE;
            }
        }

        static ActionHovering action_hovering(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
        DroneVelocity drone_velocity = action_hovering.calculateVelocity(VisionInterface::getInstance()->getBlueHx(),
                                                                         VisionInterface::getInstance()->getBlueHy());


        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }
    else if (cur_action_id == ADJUSTALTITUDE)
    {
        double_t altitude_target =
                vwpp::DynamicRecfgInterface::getInstance()->getLandingAltitude();


        if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <=
            vwpp::DynamicRecfgInterface::getInstance()->getAltitudeToleranceError())
        {
            static vwpp::JudgeAchieveCounter
                    judge_achieve_counter(
                    vwpp::DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
            if (judge_achieve_counter.isAchieve())
            {
                p_task_base->task_state = TASK_FINISH;
                return 1;
            }
        }

        static ActionAdjustAltitude action_adjust_altitude;
        DroneVelocity drone_velocity = action_adjust_altitude.calculateVelocity(altitude_target,
                                                                                PX4Interface::getInstance()->getCurZ());

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);

    }

    p_task_base->task_state = TASK_PROCESSING;
    return 0;
}


vwpp::TaskTakeoff::TaskTakeoff()
{
    p_task_base = new TaskBase(TAKEOFF);

    p_task_base->task_state = TASK_START;

    cur_action_id = ADJUSTALTITUDE;
}


vwpp::TaskTakeoff::~TaskTakeoff()
{
    delete p_task_base;
}


vwpp::TaskID vwpp::TaskTakeoff::getTaskID()
{
    return p_task_base->task_id;
}


vwpp::TaskState vwpp::TaskTakeoff::getTaskState()
{
    return p_task_base->task_state;
}


int8_t vwpp::TaskTakeoff::run()
{
    // ROS_INFO("Task: Takeoff!");
    // VisionInterface::getInstance()->update();
    // PX4Interface::getInstance()->update();


    if (cur_action_id == ADJUSTALTITUDE)
    {
        double_t altitude_target =
                DynamicRecfgInterface::getInstance()->getNormalFlightAltitude();

        if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <=
            DynamicRecfgInterface::getInstance()->getAltitudeToleranceError())
        {
            static vwpp::JudgeAchieveCounter
                    judge_achieve_counter(
                    vwpp::DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());

            if (judge_achieve_counter.isAchieve())
            {
                p_task_base->task_state = TASK_FINISH;
                return 1;
            }
        }

        static ActionAdjustAltitude action_adjust_altitude;
        DroneVelocity drone_velocity =
                action_adjust_altitude.calculateVelocity(altitude_target,
                                                         PX4Interface::getInstance()->getCurZ());


        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

        PX4Interface::getInstance()->publishLocalVel(cmd_vel);
    }

    p_task_base->task_state = TASK_PROCESSING;
    return 0;

}

