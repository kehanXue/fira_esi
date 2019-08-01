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


vwpp::TaskAvoidance::TaskAvoidance()
{
    p_task_base = new TaskBase(AVOIDANCE);

    cur_action_id = ADJUSTALTITUDE;

    p_task_base->task_state = TASK_START;

    p_action_adjust_altitude = new ActionAdjustAltitude();

    altitude_target = 0.0;

    forward_counter = 0;
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

    // VisionInterface::getInstance()->update();
    // PX4Interface::getInstance()->update();

    // geometry_msgs::Twist cmd_vel;
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


            // TODO initial_x, y
            // TODO ptr
            TargetPosXYZYaw target_pos_xyz_yaw = p_action_adjust_altitude->calculateVelocity(this->altitude_target,
                                                                                             PX4Interface::getInstance()->getCurZ());

            // cmd_vel.linear.x = drone_velocity.x;
            // cmd_vel.linear.y = drone_velocity.y;
            // cmd_vel.linear.z = drone_velocity.z;
            // cmd_vel.angular.z = drone_velocity.yaw;
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
        else if (inter_adjust_altitude_time == 2)
        {
            // TODO initial_x, y
            static ActionAdjustAltitude action_adjust_altitude;
            TargetPosXYZYaw target_pos_xyz_yaw = action_adjust_altitude.calculateVelocity(this->altitude_target,
                                                                                          PX4Interface::getInstance()->getCurZ());

            // cmd_vel.linear.x = drone_velocity.x;
            // cmd_vel.linear.y = drone_velocity.y;
            // cmd_vel.linear.z = drone_velocity.z;
            // cmd_vel.angular.z = drone_velocity.yaw;
            PX4Interface::getInstance()->publishTarget(target_pos_xyz_yaw);

            if (fabs(PX4Interface::getInstance()->getCurZ() - altitude_target) <=
                vwpp::DynamicRecfgInterface::getInstance()->getAltitudeTolerance())
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
            this->resetAdjustAltitudeOnXY(PX4Interface::getInstance()->getCurX(),
                                          PX4Interface::getInstance()->getCurY(),
                                          PX4Interface::getInstance()->getCurYaw());
            this->altitude_target =
                    vwpp::DynamicRecfgInterface::getInstance()->getNormalFlightAltitude();
            inter_adjust_altitude_time += 1;
        }

        // static ActionTrackingLine action_tracking_line(this->altitude_target);
        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                p_action_tracking_line->calculateVelocity(
                        vwpp::VisionInterface::getInstance()->getLineOffset(),
                        vwpp::VisionInterface::getInstance()->getLineRotation());


        PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);
        // cmd_vel.linear.x = drone_velocity.x;
        // cmd_vel.linear.y = drone_velocity.y;
        // cmd_vel.linear.z = drone_velocity.z;
        // cmd_vel.angular.z = drone_velocity.yaw;

    }

    // PX4Interface::getInstance()->publishSetpointVel(cmd_vel);
    p_task_base->task_state = TASK_PROCESSING;
    return 0;

}


int8_t vwpp::TaskAvoidance::resetAdjustAltitudeOnXY(double_t _hover_x, double_t _hover_y, double_t _hold_yaw)
{
    p_action_adjust_altitude->setAdjustAltitudeXYYaw(_hover_x, _hover_y, _hold_yaw);

    return 0;
}


vwpp::TaskHoverOnQR::TaskHoverOnQR() :
        action_rotating(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude())
{
    p_task_base = new TaskBase(HOVERONQR);

    // cur_action_id = HOVERING;
    cur_action_id = ROTATION;

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
    // static std::string last_qr_inform;
    // std::string _qr_inform = VisionInterface::getInstance()->getGroundQRinform();
    ROS_INFO("QR inform: %s", _qr_inform.c_str());
    // if (_qr_inform.empty())
    // {
    //     _qr_inform = last_qr_inform;
    // }
    // last_qr_inform = _qr_inform;

    static int inter_hovering_time = 1;

    if (cur_action_id == HOVERING)
    {
        ROS_INFO("Current task HoverOnQR action: HOVERING");
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
                    ROS_INFO("Task HoverOnQR switch action to ROTATION!");
                }
            }
            if (inter_hovering_time == 2)
            {

                p_task_base->task_state = TASK_FINISH;
                inter_hovering_time = 1;

                return _qr_inform.at(_qr_inform.size() - 1);

                // static JudgeAchieveCounter judge_achieve_counter(
                //         DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
                // if (judge_achieve_counter.isAchieve())
                // {
                //     p_task_base->task_state = TASK_FINISH;
                //     inter_hovering_time = 1;
                //
                //     return _qr_inform.at(_qr_inform.size() - 1);
                // }
            }
        }

        static ActionHovering action_hovering(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());

        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                action_hovering.calculateVelocity(VisionInterface::getInstance()->getQRxOffset(),
                                                  VisionInterface::getInstance()->getQRyOffset());

        ROS_ERROR("QR x_offset:%lf, QR y_offset:%lf",
                  VisionInterface::getInstance()->getQRxOffset(),
                  VisionInterface::getInstance()->getQRyOffset());

        PX4Interface::getInstance()->publishTarget(target_vel_xy_pos_z_yaw);
        // geometry_msgs::Twist cmd_vel;
        // cmd_vel.linear.x = drone_velocity.x;
        // cmd_vel.linear.y = drone_velocity.y;
        // cmd_vel.linear.z = drone_velocity.z;
        // cmd_vel.angular.z = drone_velocity.yaw;

        // PX4Interface::getInstance()->publishSetpointVel(cmd_vel);

    }
    else if (cur_action_id == ROTATION)
    {
        ROS_INFO("Current task HoverOnQR action: ROTATION");
        // TODO
        Direction target_direction = LOCAL_FORWARD;         // Init
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
                // yaw_target = M_PI * 1.5;
                yaw_target = -M_PI / 2.;
                break;
        }

        ROS_ERROR("On QR rotating to %lf", yaw_target);
        ROS_ERROR("Current yaw: %lf", PX4Interface::getInstance()->getCurYaw());
        ROS_ERROR("Yaw theta: %lf", (fmod(fabs(yaw_target - PX4Interface::getInstance()->getCurYaw()), 2 * M_PI)));
        // TODO
        if (fmod(fabs(yaw_target - PX4Interface::getInstance()->getCurYaw()), 2 * M_PI) <=
            vwpp::DynamicRecfgInterface::getInstance()->getRotateYawTolerance() * M_PI / 180.)
        {
            ROS_INFO("Get in +1!");
            static JudgeAchieveCounter judge_achieve_counter(
                    DynamicRecfgInterface::getInstance()->getJudgeAchieveCounterThreshold());
            if (judge_achieve_counter.isAchieve())
            {
                // cur_action_id = HOVERING;
                cur_action_id = ROTATION;
                inter_hovering_time += 1;
                p_task_base->task_state = TASK_FINISH;
                ROS_WARN("Rotation task finished!");
                inter_hovering_time = 1;
                return _qr_inform.at(_qr_inform.size() - 1);
            }
        }
        // action_rotating(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());


        // TODO!
        TargetPosXYZYaw target_pos_xyz_yaw = action_rotating.calculateVelocity(yaw_target,
                                                                               PX4Interface::getInstance()->getCurYaw());

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
    action_rotating.setRotatingOnXY(_hover_x, _hover_y);
    return 0;
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

    if (cur_action_id == TRACKINGLINE)
    {
        if (VisionInterface::getInstance()->getRedXState())
        {
            cur_action_id = HOVERING;
        }

        static ActionTrackingLine action_tracking_line(
                DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());

        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                action_tracking_line.calculateVelocity(
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
        // TODO

        static ActionHovering action_hovering(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                action_hovering.calculateVelocity(VisionInterface::getInstance()->getRedXx(),
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
        // TODO
        // Action::getInstance()->openClaw();

        cur_action_id = ROTATION;

        // TODO Yaw Control.
        back_toward_yaw = (PX4Interface::getInstance()->getCurYaw() + M_PI);
    }
    else if (cur_action_id == ROTATION)
    {

        // TODO Maybe problems
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

        static ActionRotating action_rotating(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
        TargetPosXYZYaw target_pos_xyz_yaw =
                action_rotating.calculateVelocity(back_toward_yaw,
                                                  PX4Interface::getInstance()->getCurYaw());

        PX4Interface::getInstance()->publishTarget(target_pos_xyz_yaw);
        // geometry_msgs::Twist cmd_vel;
        // cmd_vel.linear.x = drone_velocity.x;
        // cmd_vel.linear.y = drone_velocity.y;
        // cmd_vel.linear.z = drone_velocity.z;
        // cmd_vel.angular.z = drone_velocity.yaw;

        // PX4Interface::getInstance()->publishSetpointVel(cmd_vel);
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

        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw =
                action_tracking_line.calculateVelocity(VisionInterface::getInstance()->getLineOffset(),
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

        // TODO
        static ActionHovering action_hovering(DynamicRecfgInterface::getInstance()->getNormalFlightAltitude());
        TargetVelXYPosZYaw target_vel_xy_pos_z_yaw
                = action_hovering.calculateVelocity(VisionInterface::getInstance()->getBlueHx(),
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

        static ActionAdjustAltitude action_adjust_altitude;
        TargetPosXYZYaw target_pos_xyz_yaw
                = action_adjust_altitude.calculateVelocity(altitude_target,
                                                           PX4Interface::getInstance()->getCurZ());

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
        TargetPosXYZYaw target_pos_xyz_yaw =
                action_adjust_altitude.calculateVelocity(altitude_target,
                                                         PX4Interface::getInstance()->getCurZ());

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

