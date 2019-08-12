//
// Created by kehan on 19-7-17.
//

#include "PX4Interface.h"
#include <utility>

using namespace vwpp;


PX4Interface::PX4Interface() :
        nh("~"),
        loop_rate(20.0)
{
    px4_state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 1, &PX4Interface::px4_state_cb, this);

    px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    px4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");

    px4_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 1, &PX4Interface::px4_pose_cb, this);

    px4_setpoint_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);

    px4_setpoint_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);

    px4_setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 1);

    // Wait for FCU connection
    while (ros::ok() && !px4_cur_state.connected)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->px4_offb_set_mode.request.custom_mode = "OFFBOARD";
    this->px4_arm_cmd.request.value = true;

    // TODO Send a few setpoints before starting
    ROS_INFO("PX4Interface initialize successfully!");
}


PX4Interface* PX4Interface::instance = nullptr;

boost::mutex PX4Interface::mutex_instance;


PX4Interface* PX4Interface::getInstance()
{
    if (instance == nullptr)
    {
        boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
        if (instance == nullptr)
        {
            instance = new PX4Interface();
        }
    }

    return instance;
}


// TODO Why?
PX4Interface::PX4Interface(const PX4Interface &, ros::Rate _loop_rate) :
        loop_rate(std::move(_loop_rate))
{

}


PX4Interface &PX4Interface::operator=(const PX4Interface &)
{

}


PX4Interface::~PX4Interface()
{
    delete instance;
}


int8_t PX4Interface::update()
{
    try
    {
        ros::spinOnce();
    }
    catch (ros::Exception &ex)
    {
        ROS_ERROR("PX4Interface Update Error: %s", ex.what());
        return -1;
    }

    return 0;
}


int8_t PX4Interface::switchOffboard()
{
    static ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (px4_cur_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (px4_set_mode_client.call(px4_offb_set_mode) &&
                px4_offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                break;
            }
            last_request = ros::Time::now();
        }

        // Must provide a pose msg! Or the plane couldn't takeoff!
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = 0;
        temp_pose.pose.position.y = 0;
        temp_pose.pose.position.z = 1;

        vwpp::PX4Interface::getInstance()->publishSetpointPose(temp_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


int8_t PX4Interface::unlockVehicle()
{
    static ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (!px4_cur_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (px4_arming_client.call(px4_arm_cmd) &&
                px4_arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                break;
            }
            last_request = ros::Time::now();
        }

        // Must provide a pose msg! Or the plane couldn't takeoff!
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = 0;
        temp_pose.pose.position.y = 0;
        temp_pose.pose.position.z = 1;
        vwpp::PX4Interface::getInstance()->publishSetpointPose(temp_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


double_t PX4Interface::getCurYaw()
{
    tf::Quaternion cur_pose_quat;
    tf::quaternionMsgToTF(this->px4_cur_pose.pose.orientation, cur_pose_quat);

    double_t roll;
    double_t pitch;
    double_t yaw;

    tf::Matrix3x3(cur_pose_quat).getRPY(roll, pitch, yaw);

    return yaw;
}


double_t PX4Interface::getCurX()
{
    return this->px4_cur_pose.pose.position.x;
}


double_t PX4Interface::getCurY()
{
    return this->px4_cur_pose.pose.position.y;
}


double_t PX4Interface::getCurZ()
{
    return this->px4_cur_pose.pose.position.z;
}


void PX4Interface::px4_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    this->px4_cur_state = *msg;
}


void PX4Interface::px4_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    this->px4_cur_pose = *msg;

}


int8_t PX4Interface::publishSetpointVel(const geometry_msgs::Twist &_vel)
{
    boost::unique_lock<boost::mutex> uq_lock_vel(mutex_cmd_pub);

    ROS_INFO("%lf,%lf,%lf,%lf", _vel.linear.x, _vel.linear.y, _vel.linear.z, _vel.angular.z);
    this->px4_setpoint_vel_pub.publish(_vel);

    return 0;
}


int8_t PX4Interface::publishSetpointPose(const geometry_msgs::PoseStamped &_pose)
{
    boost::unique_lock<boost::mutex> uq_lock_pose(mutex_cmd_pub);
    this->px4_setpoint_pose_pub.publish(_pose);

    return 0;
}


int8_t PX4Interface::publishTarget(const TargetPosXYZYaw _target_pos_xyz_yaw)
{

    mavros_msgs::PositionTarget position_target;
    position_target.coordinate_frame = position_target.FRAME_LOCAL_NED;
    position_target.type_mask =
            position_target.IGNORE_AFX | position_target.IGNORE_AFY | position_target.IGNORE_AFZ |
            // position_target.IGNORE_PX | position_target.IGNORE_PY | position_target.IGNORE_PZ |
            position_target.IGNORE_VZ | position_target.IGNORE_VY | position_target.IGNORE_VZ |
            position_target.IGNORE_YAW_RATE;

    position_target.position.x = _target_pos_xyz_yaw.px;
    position_target.position.y = _target_pos_xyz_yaw.py;
    position_target.position.z = _target_pos_xyz_yaw.pz;
    position_target.yaw = _target_pos_xyz_yaw.yaw;

    boost::unique_lock<boost::mutex> uq_lock_raw(mutex_cmd_pub);
    this->px4_setpoint_raw_pub.publish(position_target);
    return 0;
}


int8_t PX4Interface::publishTarget(const TargetVelXYPosZYaw _target_vel_xy_pos_z_yaw)
{
    mavros_msgs::PositionTarget position_target;
    position_target.coordinate_frame = position_target.FRAME_LOCAL_NED;
    position_target.type_mask =
            position_target.IGNORE_AFX | position_target.IGNORE_AFY | position_target.IGNORE_AFZ |
            position_target.IGNORE_PX | position_target.IGNORE_PY | // position_target.IGNORE_PZ |
            // position_target.IGNORE_VZ | position_target.IGNORE_VY | position_target.IGNORE_VZ |
            position_target.IGNORE_YAW_RATE;

    position_target.velocity.x = _target_vel_xy_pos_z_yaw.vx;
    position_target.velocity.y = _target_vel_xy_pos_z_yaw.vy;
    position_target.position.z = _target_vel_xy_pos_z_yaw.pz;
    position_target.yaw = _target_vel_xy_pos_z_yaw.yaw;

    ROS_WARN("vel x: %lf, vel y: %lf, vel z: %lf", position_target.velocity.x,
             position_target.velocity.y, position_target.velocity.z);

    boost::unique_lock<boost::mutex> uq_lock_raw(mutex_cmd_pub);
    this->px4_setpoint_raw_pub.publish(position_target);
    return 0;
}


int8_t PX4Interface::publishTarget(const TargetVelXYYawPosZ _target_vel_xy_yaw_pos_z)
{
    mavros_msgs::PositionTarget position_target;
    position_target.coordinate_frame = position_target.FRAME_LOCAL_NED;
    position_target.type_mask =
            position_target.IGNORE_AFX | position_target.IGNORE_AFY | position_target.IGNORE_AFZ |
            position_target.IGNORE_PX | position_target.IGNORE_PY | // position_target.IGNORE_PZ |
            // position_target.IGNORE_VZ | position_target.IGNORE_VY | position_target.IGNORE_VZ |
            position_target.IGNORE_YAW;

    position_target.velocity.x = _target_vel_xy_yaw_pos_z.vx;
    position_target.velocity.y = _target_vel_xy_yaw_pos_z.vy;
    position_target.position.z = _target_vel_xy_yaw_pos_z.pz;
    position_target.yaw_rate = _target_vel_xy_yaw_pos_z.yaw_rate;
    ROS_WARN("vel x: %lf, vel y: %lf, yaw_rate: %lf", position_target.velocity.x,
             position_target.velocity.y, position_target.yaw_rate);

    boost::unique_lock<boost::mutex> uq_lock_raw(mutex_cmd_pub);
    this->px4_setpoint_raw_pub.publish(position_target);

    return 0;
}


int8_t PX4Interface::publishTarget(const TargetPosXYZVelYaw _target_pos_xyz_vel_yaw)
{
    mavros_msgs::PositionTarget position_target;
    position_target.coordinate_frame = position_target.FRAME_LOCAL_NED;
    position_target.type_mask =
            position_target.IGNORE_AFX | position_target.IGNORE_AFY | position_target.IGNORE_AFZ |
            position_target.IGNORE_VZ | position_target.IGNORE_VY | position_target.IGNORE_VZ |
            position_target.IGNORE_YAW;

    position_target.position.x = _target_pos_xyz_vel_yaw.px;
    position_target.position.y = _target_pos_xyz_vel_yaw.py;
    position_target.position.z = _target_pos_xyz_vel_yaw.pz;
    position_target.yaw_rate = _target_pos_xyz_vel_yaw.yaw_rate;

    boost::unique_lock<boost::mutex> uq_lock_raw(mutex_cmd_pub);
    this->px4_setpoint_raw_pub.publish(position_target);

    return 0;
}


