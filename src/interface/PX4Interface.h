//
// Created by kehan on 19-7-17.
//

#ifndef FIRA_ESI_PX4INTERFACE_H_
#define FIRA_ESI_PX4INTERFACE_H_



#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

#include <boost/thread.hpp>

namespace vwpp
{
    struct TargetVelXYPosZYaw
    {
        double_t vx;
        double_t vy;
        double_t pz;
        double_t yaw;
    };

    struct TargetPosXYZYaw
    {
        double_t px;
        double_t py;
        double_t pz;
        double_t yaw;
    };

    class PX4Interface
    {
    public:

        static PX4Interface* getInstance();

        virtual ~PX4Interface();

        int8_t update();

        int8_t switchOffboard();

        int8_t unlockVehicle();

        double_t getCurYaw();

        double_t getCurX();

        double_t getCurY();

        double_t getCurZ();

        int8_t publishSetpointVel(const geometry_msgs::Twist &_vel);

        int8_t publishSetpointPose(const geometry_msgs::PoseStamped &_pose);

        int8_t publishTarget(const TargetPosXYZYaw _target_pos_xyz_yaw);

        int8_t publishTarget(const TargetVelXYPosZYaw _target_vel_xy_pos_z_yaw);

    private:

        PX4Interface();

        PX4Interface(const PX4Interface &, ros::Rate _loop_rate);

        PX4Interface &operator=(const PX4Interface &);

        static PX4Interface* instance;
        static boost::mutex mutex_instance;

        void px4_state_cb(const mavros_msgs::State::ConstPtr &msg);

        void px4_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);


        ros::NodeHandle nh;
        ros::Rate loop_rate;

        ros::Subscriber px4_state_sub;
        ros::ServiceClient px4_arming_client;
        ros::ServiceClient px4_set_mode_client;
        ros::Subscriber px4_pose_sub;
        ros::Publisher px4_setpoint_vel_pub;
        ros::Publisher px4_setpoint_pose_pub;
        ros::Publisher px4_setpoint_raw_pub;

        boost::mutex mutex_cmd_pub;

        mavros_msgs::State px4_cur_state;
        mavros_msgs::SetMode px4_offb_set_mode;
        mavros_msgs::CommandBool px4_arm_cmd;

        geometry_msgs::PoseStamped px4_cur_pose;

    };

}



#endif //FIRA_ESI_PX4INTERFACE_H_

