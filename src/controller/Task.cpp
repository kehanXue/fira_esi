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


/*****************************************************
 * Task: Take off
 ****************************************************/
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
    ROS_INFO("Task: Takeoff!");
    if (cur_action_id == ADJUSTALTITUDE)
    {
        double_t altitude_target =
                DynamicRecfgInterface::getInstance()->getNormalFlightAltitude();

        if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <=
            DynamicRecfgInterface::getInstance()->getAltitudeTolerance())
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

        PX4Interface::getInstance()->publishSetpointVel(cmd_vel);
    }

    p_task_base->task_state = TASK_PROCESSING;
    return 0;

}


/*****************************************************
 * Task: Navigation
 ****************************************************/
vwpp::TaskNavigation::TaskNavigation() :
        runtime(0),
        cur_action_id(TRACKINGLINE)
{
    p_task_base = new TaskBase(NAVIGATION);
    p_task_base->task_state = TASK_START;
    p_action_tracking_line = new ActionTrackingLine(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
}


vwpp::TaskNavigation::~TaskNavigation()
{
    delete p_task_base;
    delete p_action_tracking_line;
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
    runtime++;
    if (runtime >=
        DynamicRecfgInterface::getInstance()->getNavigationPerSustainTime())
    {
        p_task_base->task_state = TASK_FINISH;
        ROS_INFO("Got +1");
    }
    else
    {
        p_task_base->task_state = TASK_PROCESSING;
    }

    if (cur_action_id == TRACKINGLINE)
    {


        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                p_action_tracking_line->calculateVelocity(
                        vwpp::VisionInterface::getInstance()->getLineOffset(),
                        vwpp::VisionInterface::getInstance()->getLineRotation());

        PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);
    }

    return 0;
}


int8_t vwpp::TaskNavigation::restart()
{
    runtime = 0;

    return 0;
}


/*****************************************************
 * Task: Avoidance
 ****************************************************/
vwpp::TaskAvoidance::TaskAvoidance() :
        cur_action_id(ADJUSTALTITUDE),
        altitude_target(0.),
        forward_counter(0),
        inter_adjust_altitude_time(FIRST_IN)
{
    p_task_base = new TaskBase(AVOIDANCE);

    p_task_base->task_state = TASK_START;

    p_action_adjust_altitude = new ActionAdjustAltitude();
}


vwpp::TaskAvoidance::~TaskAvoidance()
{
    delete p_task_base;
    delete p_action_adjust_altitude;
    delete p_action_tracking_line;
}


vwpp::TaskID vwpp::TaskAvoidance::getTaskID()
{
    return p_task_base->task_id;
}


vwpp::TaskState vwpp::TaskAvoidance::getTaskState()
{
    return p_task_base->task_state;
}


// Has bug! static!
int8_t vwpp::TaskAvoidance::run(GateType _gate_type)
{
    if (cur_action_id == ADJUSTALTITUDE)
    {

        if (inter_adjust_altitude_time == FIRST_IN)
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


            TargetPosXYZYaw target_pos_xyz_yaw =
                    p_action_adjust_altitude->calculateVelocity(this->altitude_target);
            PX4Interface::getInstance()->publishTarget(target_pos_xyz_yaw);


            if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <=
                vwpp::DynamicRecfgInterface::getInstance()->getAltitudeTolerance())
            {
                static JudgeAchieveCounter judge_achieve_counter(
                        DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
                if (judge_achieve_counter.isAchieve())
                {
                    cur_action_id = TRACKINGLINE;
                    p_action_tracking_line = new ActionTrackingLine(this->altitude_target);
                }
            }

        }
        else if (inter_adjust_altitude_time == SECOND_IN)
        {
            TargetPosXYZYaw target_pos_xyz_yaw = p_action_adjust_altitude->calculateVelocity(this->altitude_target);
            PX4Interface::getInstance()->publishTarget(target_pos_xyz_yaw);

            if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <=
                vwpp::DynamicRecfgInterface::getInstance()->getAltitudeTolerance())
            {
                static JudgeAchieveCounter judge_achieve_counter(
                        DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
                if (judge_achieve_counter.isAchieve())
                {

                    p_task_base->task_state = TASK_FINISH;
                    inter_adjust_altitude_time = FIRST_IN;
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
            this->resetAdjustAltitudeOnXYYaw(PX4Interface::getInstance()->getCurX(),
                                             PX4Interface::getInstance()->getCurY(),
                                             PX4Interface::getInstance()->getCurYaw());
            // TODO
            this->altitude_target =
                    vwpp::DynamicRecfgInterface::getInstance()->getNormalFlightAltitude();
            inter_adjust_altitude_time = SECOND_IN;
        }

        // static ActionTrackingLine action_tracking_line(this->altitude_target);
        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                p_action_tracking_line->calculateVelocity(
                        vwpp::VisionInterface::getInstance()->getLineOffset(),
                        vwpp::VisionInterface::getInstance()->getLineRotation());


        PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);
    }

    p_task_base->task_state = TASK_PROCESSING;
    return 0;

}


int8_t vwpp::TaskAvoidance::resetAdjustAltitudeOnXYYaw(double_t _hover_x, double_t _hover_y, double_t _hold_yaw)
{
    p_action_adjust_altitude->setAdjustAltitudeXYYaw(_hover_x, _hover_y, _hold_yaw);

    return 0;
}


/*****************************************************
 * Task: Hover on QR
 ****************************************************/
vwpp::TaskHoverOnQR::TaskHoverOnQR() :
        action_rotating(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude()),
        cur_action_id(ROTATION)
{
    p_task_base = new TaskBase(HOVERONQR);
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


char vwpp::TaskHoverOnQR::run(TaskID _cur_task_id, std::string _qr_inform)
{
    ROS_INFO("QR inform: %s", _qr_inform.c_str());

    if (cur_action_id == ROTATION)
    {
        ROS_INFO("Current task HoverOnQR action: ROTATION");
        Direction target_direction = LOCAL_FORWARD;
        ROS_ERROR("Current task id: %d", _cur_task_id);
        switch (_qr_inform.at(_cur_task_id))
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
                yaw_target = -M_PI / 2.;
                break;
        }

        ROS_ERROR("On QR rotating to %lf", yaw_target);
        ROS_ERROR("Current yaw: %lf", PX4Interface::getInstance()->getCurYaw());
        ROS_ERROR("Yaw theta: %lf", (fmod(fabs(yaw_target - PX4Interface::getInstance()->getCurYaw()), 2 * M_PI)));

        if (fmod(fabs(yaw_target - PX4Interface::getInstance()->getCurYaw()), 2 * M_PI) <=
            vwpp::DynamicRecfgInterface::getInstance()->getRotateYawTolerance() * M_PI / 180.)
        {
            ROS_INFO("Get in +1!");
            static JudgeAchieveCounter judge_achieve_counter(
                    DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
            if (judge_achieve_counter.isAchieve())
            {
                p_task_base->task_state = TASK_FINISH;
                ROS_WARN("Rotation task finished!");
                return _qr_inform.at(_qr_inform.size() - 1);
            }
        }

        TargetPosXYZYaw target_pos_xyz_yaw = action_rotating.calculateVelocity(yaw_target);
        PX4Interface::getInstance()->publishTarget(target_pos_xyz_yaw);

        // DroneVelocity drone_velocity = action_rotating.calculateVelocity(yaw_target,
        //                                                                  PX4Interface::getInstance()->getCurYaw());

        // geometry_msgs::Twist cmd_vel;
        // cmd_vel.linear.x = drone_velocity.x;
        // cmd_vel.linear.y = drone_velocity.y;
        // cmd_vel.linear.x = 0;
        // cmd_vel.linear.y = 0;
        // cmd_vel.linear.z = drone_velocity.z;
        // cmd_vel.angular.z = drone_velocity.yaw;

        // PX4Interface::getInstance()->publishSetpointVel(cmd_vel);

    }

    p_task_base->task_state = TASK_PROCESSING;
    return _qr_inform.at(_qr_inform.size() - 1);
}


int8_t vwpp::TaskHoverOnQR::resetRotatingOnXY(double_t _hover_x, double_t _hover_y)
{
    action_rotating.resetRotatingOnXY(_hover_x, _hover_y);
    return 0;
}


/*****************************************************
 * Task: Delivering
 ****************************************************/
vwpp::TaskDelivering::TaskDelivering() :
        cur_action_id(TRACKINGLINE)
{
    p_task_base = new TaskBase(DELIVERING);
    p_task_base->task_state = TASK_START;

    p_action_tracking_line =
            new ActionTrackingLine(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
    p_action_hovering =
            new ActionHovering(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
    p_action_rotating =
            new ActionRotating(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
}


vwpp::TaskDelivering::~TaskDelivering()
{
    delete p_task_base;
    delete p_action_tracking_line;
    delete p_action_hovering;
    delete p_action_rotating;
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

    if (cur_action_id == TRACKINGLINE)
    {
        ROS_WARN("RedX State: %d",
                 VisionInterface::getInstance()->getRedXState());

        if (VisionInterface::getInstance()->getRedXState())
        {
            p_action_hovering->resetTargetYaw(PX4Interface::getInstance()->getCurYaw());
            cur_action_id = HOVERING;
            ROS_WARN("Action switch to HOVERING");
        }

        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                p_action_tracking_line->calculateVelocity(
                        vwpp::VisionInterface::getInstance()->getLineOffset(),
                        vwpp::VisionInterface::getInstance()->getLineRotation());

        PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);
        // geometry_msgs::Twist cmd_vel;
        // cmd_vel.linear.x = drone_velocity.x;
        // cmd_vel.linear.y = drone_velocity.y;
        // cmd_vel.linear.z = drone_velocity.z;
        // cmd_vel.angular.z = drone_velocity.yaw;
        //
        // PX4Interface::getInstance()->publishSetpointVel(cmd_vel);
    }
    else if (cur_action_id == HOVERING)
    {
        ROS_ERROR("Current RedX: %lf, RedY: %lf",
                  VisionInterface::getInstance()->getRedXx(),
                  VisionInterface::getInstance()->getRedXy());

        if (fabs(VisionInterface::getInstance()->getRedXx() - 0.) <=
            vwpp::DynamicRecfgInterface::getInstance()->getRedXOffsetXTolerance() &&
            fabs(VisionInterface::getInstance()->getRedXy() - 0.) <=
            vwpp::DynamicRecfgInterface::getInstance()->getRedXOffsetYTolerance())
        {
            static JudgeAchieveCounter judge_achieve_counter(
                    DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
            if (judge_achieve_counter.isAchieve())
            {
                ROS_WARN("Action switch to OPENCLAW");
                cur_action_id = OPENCLAW;
            }
        }

        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                p_action_hovering->calculateVelocity(VisionInterface::getInstance()->getRedXx(),
                                                     VisionInterface::getInstance()->getRedXy());

        PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);
        // geometry_msgs::Twist cmd_vel;
        // cmd_vel.linear.x = drone_velocity.x;
        // cmd_vel.linear.y = drone_velocity.y;
        // cmd_vel.linear.z = drone_velocity.z;
        // cmd_vel.angular.z = drone_velocity.yaw;

        // PX4Interface::getInstance()->publishSetpointVel(cmd_vel);
    }
    else if (cur_action_id == OPENCLAW)
    {
        static int64_t runtime = 0;
        runtime++;

        std_msgs::Bool open_claw_msg;
        open_claw_msg.data = true;
        ROS_WARN("Sending open claw message");
        ClawInterface::getInstance()->publishOpenClawMsg(open_claw_msg);

        if (runtime >= DynamicRecfgInterface::getInstance()->getOpenClawMsgSendFrequency())
        {
            cur_action_id = ROTATION;
            p_action_rotating->resetRotatingOnXY(PX4Interface::getInstance()->getCurX(),
                                                 PX4Interface::getInstance()->getCurY());
            ROS_WARN("Action switch to ROTATION");
            back_toward_yaw = (PX4Interface::getInstance()->getCurYaw() + M_PI);
        }


        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                p_action_hovering->calculateVelocity(VisionInterface::getInstance()->getRedXx(),
                                                     VisionInterface::getInstance()->getRedXy());

        PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);

    }
    else if (cur_action_id == ROTATION)
    {
        if (fmod(fabs(PX4Interface::getInstance()->getCurYaw() - back_toward_yaw), 2 * M_PI) <=
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

        // TargetPosXYZYaw target_pos_xyz_yaw =
        //         p_action_rotating->calculateVelocity(back_toward_yaw);

        // PX4Interface::getInstance()->publishTarget(target_pos_xyz_yaw);

        DroneVelocity drone_velocity =
                p_action_rotating->calculateVelocity(back_toward_yaw, PX4Interface::getInstance()->getCurYaw());
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = drone_velocity.x;
        cmd_vel.linear.y = drone_velocity.y;
        cmd_vel.linear.z = drone_velocity.z;
        cmd_vel.angular.z = drone_velocity.yaw;

        PX4Interface::getInstance()->publishSetpointVel(cmd_vel);
    }

    p_task_base->task_state = TASK_PROCESSING;
    return 0;
}


/*****************************************************
 * Task: Landing
 ****************************************************/
vwpp::TaskLanding::TaskLanding() :
        cur_action_id(TRACKINGLINE)
{
    p_task_base = new TaskBase(LANDING);
    p_task_base->task_state = TASK_START;

    p_action_tracking_line =
            new ActionTrackingLine(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
    p_action_hovering =
            new ActionHovering(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
    p_action_adjust_altitude =
            new ActionAdjustAltitude();
}


vwpp::TaskLanding::~TaskLanding()
{
    delete p_task_base;
    delete p_action_tracking_line;
    delete p_action_hovering;
    delete p_action_adjust_altitude;
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
    if (cur_action_id == TRACKINGLINE)
    {
        if (VisionInterface::getInstance()->getBlueHState())
        {
            p_action_hovering->resetTargetYaw(PX4Interface::getInstance()->getCurYaw());
            cur_action_id = HOVERING;
            ROS_WARN("Action switch to HOVERING");
        }

        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                p_action_tracking_line->calculateVelocity(
                        VisionInterface::getInstance()->getLineOffset(),
                        VisionInterface::getInstance()->getLineRotation());

        PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);
        // geometry_msgs::Twist cmd_vel;
        // cmd_vel.linear.x = drone_velocity.x;
        // cmd_vel.linear.y = drone_velocity.y;
        // cmd_vel.linear.z = drone_velocity.z;
        // cmd_vel.angular.z = drone_velocity.yaw;
        //
        // PX4Interface::getInstance()->publishSetpointVel(cmd_vel);
    }
    else if (cur_action_id == HOVERING)
    {
        ROS_ERROR("Current BlueX: %lf, BlueY: %lf",
                  VisionInterface::getInstance()->getBlueHx(),
                  VisionInterface::getInstance()->getBlueHy());

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
                p_action_adjust_altitude->setAdjustAltitudeXYYaw(PX4Interface::getInstance()->getCurX(),
                                                                 PX4Interface::getInstance()->getCurY(),
                                                                 PX4Interface::getInstance()->getCurYaw());
            }
        }

        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                p_action_hovering->calculateVelocity(VisionInterface::getInstance()->getBlueHx(),
                                                     VisionInterface::getInstance()->getBlueHy());

        PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);

        // geometry_msgs::Twist cmd_vel;
        // cmd_vel.linear.x = drone_velocity.x;
        // cmd_vel.linear.y = drone_velocity.y;
        // cmd_vel.linear.z = drone_velocity.z;
        // cmd_vel.angular.z = drone_velocity.yaw;
        //
        // PX4Interface::getInstance()->publishSetpointVel(cmd_vel);
    }
    else if (cur_action_id == ADJUSTALTITUDE)
    {
        double_t altitude_target =
                vwpp::DynamicRecfgInterface::getInstance()->getLandingAltitude();


        if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <=
            vwpp::DynamicRecfgInterface::getInstance()->getAltitudeTolerance())
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

        TargetPosXYZYaw target_pos_xyz_yaw
                = p_action_adjust_altitude->calculateVelocity(altitude_target);

        PX4Interface::getInstance()->publishTarget(target_pos_xyz_yaw);
        // geometry_msgs::Twist cmd_vel;
        // cmd_vel.linear.x = drone_velocity.x;
        // cmd_vel.linear.y = drone_velocity.y;
        // cmd_vel.linear.z = drone_velocity.z;
        // cmd_vel.angular.z = drone_velocity.yaw;
        //
        // PX4Interface::getInstance()->publishSetpointVel(cmd_vel);

    }

    p_task_base->task_state = TASK_PROCESSING;
    return 0;
}



