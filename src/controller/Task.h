//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_TASK_H_
#define FIRA_ESI_TASK_H_

#include <cstdint>
#include <vector>

#include "vision/mycv.h"
#include "controller/Action.h"
#include "interface/VisionInterface.h"
#include "interface/PX4Interface.h"
#include "interface/DynamicRecfgInterface.h"
#include <interface/ClawInterface.h>
#include "utils/JudgeAchieveCounter.h"

namespace vwpp
{

    enum Direction
    {
        LOCAL_FORWARD = 0,
        LOCAL_LEFT,
        LOCAL_BACK,
        LOCAL_RIGHT
    };

    enum TaskID
    {
        TAKEOFF = -4,
        NAVIGATION = -3,
        AVOIDANCE = -2,
        HOVERONQR = -1,
        DELIVERING = 0,
        SCANTOWER = 1,
        SCANBUILDING = 2,
        LANDING = 3
    };

    enum TaskState
    {
        TASK_SUSPEND,
        TASK_START,
        TASK_PROCESSING = 0,
        TASK_FINISH
    };

    enum ActionRuntime
    {
        FIRST_IN = 1,
        SECOND_IN
    };

    class TaskBase
    {
    public:

        TaskBase();

        explicit TaskBase(TaskID _task_id);

        virtual ~TaskBase();

        TaskID task_id;
        TaskState task_state;

    private:

        ros::NodeHandle nh;
    };

    class TaskTakeoff
    {
    public:

        TaskTakeoff();

        virtual ~TaskTakeoff();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run();

    private:

        TaskBase* p_task_base;

        ActionID cur_action_id;

    };

    class TaskNavigation
    {
    public:

        TaskNavigation();

        virtual ~TaskNavigation();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t restart();

        int8_t run();

    private:

        TaskBase* p_task_base;

        ActionTrackingLine* p_action_tracking_line;

        ActionID cur_action_id;

        int64_t runtime;

    };


    enum GateType
    {
        YELLOW = 0,
        RED,
        NONE
    };

    class TaskAvoidance
    {
    public:

        TaskAvoidance();

        virtual ~TaskAvoidance();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run(GateType _gate_type);

        int8_t resetAdjustAltitudeOnXYYaw(double_t _hover_x, double_t _hover_y, double_t _hold_yaw);

    private:

        TaskBase* p_task_base;

        ActionID cur_action_id;

        double_t altitude_target;

        ActionAdjustAltitude* p_action_adjust_altitude;
        ActionTrackingLine* p_action_tracking_line{};


        // TODO Timer
        int64_t forward_counter;

        ActionRuntime inter_adjust_altitude_time;
    };

    class TaskHoverOnQR
    {
    public:

        TaskHoverOnQR();

        virtual ~TaskHoverOnQR();

        TaskID getTaskID();

        TaskState getTaskState();

        char run(TaskID _cur_task_id, std::string _qr_inform);

        int8_t resetRotatingOnXY(double_t _hover_x, double_t _hover_y);

    private:
        TaskBase* p_task_base;
        ActionID cur_action_id;

        // TODO
        ActionRotating action_rotating;
        ActionTrackingLine* p_action_tracking_line;
    };

    class TaskDelivering
    {
    public:

        TaskDelivering();

        virtual ~TaskDelivering();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run();

    private:

        TaskBase* p_task_base;
        ActionID cur_action_id;

        ActionTrackingLine* p_action_tracking_line;
        ActionHovering* p_action_hovering;
        ActionRotating* p_action_rotating;
    };


    class TaskLanding
    {
    public:

        TaskLanding();

        virtual ~TaskLanding();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run();

    private:

        TaskBase* p_task_base;
        ActionID cur_action_id;

        ActionTrackingLine* p_action_tracking_line;
        ActionHovering* p_action_hovering;
        ActionAdjustAltitude* p_action_adjust_altitude;
    };

    class TaskScanTower
    {
    public:

        TaskScanTower();

        virtual ~TaskScanTower();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run(double_t _cycle_radius);

        int8_t resetAdjustAltitudeOnXYYaw(double_t _hover_x, double_t _hover_y, double_t _hold_yaw);

        int8_t setTargetPoints();

    private:

        // TODO utils
        int8_t convertPointLocal2Body(geometry_msgs::Point &_point_in, geometry_msgs::Point &_point_out);

        int64_t cycle_time_counter;
        ActionRuntime inter_adjust_altitude_time;

        TaskBase* p_task_base;
        ActionID cur_action_id;

        double_t target_altitude;
        ActionCycleMoving* p_action_cycle_moving;
        ActionRotating* p_action_rotating;
        ActionAdjustAltitude* p_action_adjust_altitude;
        ActionGoToLocalPositionHoldYaw* p_action_go_to_local_position_hold_yaw;

        std::vector<geometry_msgs::Point> vec_target_points;
        geometry_msgs::Point cur_target_point;
        u_int64_t cur_target_point_index;
        double_t cur_target_yaw;

        tf::TransformListener odom_base_tf_listener;

    };

    class TaskScanBuilding
    {
    public:
        TaskScanBuilding();

        virtual ~TaskScanBuilding();

        TaskID getTaskID();

        TaskState getTaskState();

        int8_t run();

        int8_t setTargetPoints();

    private:

        int8_t convertPointLocal2Body(geometry_msgs::Point &_point_in, geometry_msgs::Point &_point_out);

        std::vector<vwpp::TargetPosXYZYaw> vec_target_poses;
        vwpp::TargetPosXYZYaw cur_pose_target{};
        u_int64_t cur_pose_target_index;

        ActionGoToLocalPositionHoldYaw* p_action_go_to_local_position_hold_yaw;

        TaskBase* p_task_base;
        ActionID cur_action_id;

        tf::TransformListener odom_base_tf_listener;
    };
}


#endif //FIRA_ESI_TASK_H_

