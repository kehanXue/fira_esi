//
// Created by kehan on 19-7-25.
//

#include "DynamicRecfgInterface.h"


vwpp::DynamicRecfgInterface::DynamicRecfgInterface()
{

    nh = ros::NodeHandle("~");

    // this ptr.
    dyconfig_cb_type = boost::bind(&vwpp::DynamicRecfgInterface::reconfig_cb, this, _1, _2);
    dyconfig_server.setCallback(dyconfig_cb_type);
}


vwpp::DynamicRecfgInterface::DynamicRecfgInterface(const vwpp::DynamicRecfgInterface &)
{

}


vwpp::DynamicRecfgInterface &vwpp::DynamicRecfgInterface::operator=(const vwpp::DynamicRecfgInterface &)
{

}


vwpp::DynamicRecfgInterface::~DynamicRecfgInterface()
{
    delete instance;
}


vwpp::DynamicRecfgInterface* vwpp::DynamicRecfgInterface::instance = nullptr;


boost::mutex vwpp::DynamicRecfgInterface::mutex_instance;


vwpp::DynamicRecfgInterface* vwpp::DynamicRecfgInterface::getInstance()
{
    if (instance == nullptr)
    {
        boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
        if (instance == nullptr)
        {
            instance = new DynamicRecfgInterface();
        }
    }

    return instance;
}


int8_t vwpp::DynamicRecfgInterface::update()
{
    try
    {
        ros::spinOnce();
    }
    catch (ros::Exception &ex)
    {
        ROS_ERROR("DynamicRecfgInterface Update Error: %s", ex.what());
        return -1;
    }

    return 0;
}


void vwpp::DynamicRecfgInterface::reconfig_cb(fira_esi::fira_esi_dynamic_cfgConfig &_config, uint32_t _level)
{
    this->forward_vel = _config.forward_vel;

    this->normal_flight_altitude = _config.normal_flight_altitude;
    this->landing_altitude = _config.landing_altitude;


    this->pid_p_v2p_x_kp = _config.pid_p_v2p_x_kp;
    this->pid_p_v2p_x_ki = _config.pid_p_v2p_x_ki;
    this->pid_p_v2p_x_kd = _config.pid_p_v2p_x_kd;
    this->pid_p_v2p_x_has_threshold = _config.pid_p_v2p_x_has_threshold;
    this->pid_p_v2p_x_threshold = _config.pid_p_v2p_x_threshold;

    this->pid_p_v2p_y_kp = _config.pid_p_v2p_y_kp;
    this->pid_p_v2p_y_ki = _config.pid_p_v2p_y_ki;
    this->pid_p_v2p_y_kd = _config.pid_p_v2p_y_kd;
    this->pid_p_v2p_y_has_threshold = _config.pid_p_v2p_y_has_threshold;
    this->pid_p_v2p_y_threshold = _config.pid_p_v2p_y_threshold;

    this->pid_p_v2p_z_kp = _config.pid_p_v2p_z_kp;
    this->pid_p_v2p_z_ki = _config.pid_p_v2p_z_ki;
    this->pid_p_v2p_z_kd = _config.pid_p_v2p_z_kd;
    this->pid_p_v2p_z_has_threshold = _config.pid_p_v2p_z_has_threshold;
    this->pid_p_v2p_z_threshold = _config.pid_p_v2p_z_threshold;

    this->pid_p_v2p_yaw_kp = _config.pid_p_v2p_yaw_kp;
    this->pid_p_v2p_yaw_ki = _config.pid_p_v2p_yaw_ki;
    this->pid_p_v2p_yaw_kd = _config.pid_p_v2p_yaw_kd;
    this->pid_p_v2p_yaw_has_threshold = _config.pid_p_v2p_yaw_has_threshold;
    this->pid_p_v2p_yaw_threshold = _config.pid_p_v2p_yaw_threshold;


    this->pid_v_v2p_x_kp = _config.pid_v_v2p_x_kp;
    this->pid_v_v2p_x_ki = _config.pid_v_v2p_x_ki;
    this->pid_v_v2p_x_kd = _config.pid_v_v2p_x_kd;
    this->pid_v_v2p_x_has_threshold = _config.pid_v_v2p_x_has_threshold;
    this->pid_v_v2p_x_threshold = _config.pid_v_v2p_x_threshold;

    this->pid_v_v2p_y_kp = _config.pid_v_v2p_y_kp;
    this->pid_v_v2p_y_ki = _config.pid_v_v2p_y_ki;
    this->pid_v_v2p_y_kd = _config.pid_v_v2p_y_kd;
    this->pid_v_v2p_y_has_threshold = _config.pid_v_v2p_y_has_threshold;
    this->pid_v_v2p_y_threshold = _config.pid_v_v2p_y_threshold;

    this->pid_v_v2p_z_kp = _config.pid_v_v2p_z_kp;
    this->pid_v_v2p_z_ki = _config.pid_v_v2p_z_ki;
    this->pid_v_v2p_z_kd = _config.pid_v_v2p_z_kd;
    this->pid_v_v2p_z_has_threshold = _config.pid_v_v2p_z_has_threshold;
    this->pid_v_v2p_z_threshold = _config.pid_v_v2p_z_threshold;

    this->pid_v_v2p_yaw_kp = _config.pid_v_v2p_yaw_kp;
    this->pid_v_v2p_yaw_ki = _config.pid_v_v2p_yaw_ki;
    this->pid_v_v2p_yaw_kd = _config.pid_v_v2p_yaw_kd;
    this->pid_v_v2p_yaw_has_threshold = _config.pid_v_v2p_yaw_has_threshold;
    this->pid_v_v2p_yaw_threshold = _config.pid_v_v2p_yaw_threshold;


    this->altitude_when_red_gate = _config.altitude_when_red_gate;
    this->altitude_when_yellow_gate = _config.altitude_when_yellow_gate;
    this->avoidance_forward_time = _config.avoidance_forward_time;

    this->altitude_tolerance = _config.altitude_tolerance;
    this->rotate_yaw_tolerance = _config.rotate_yaw_tolerance;


    this->blue_h_offset_x_tolerance = _config.blue_h_offset_x_tolerance;
    this->blue_h_offset_y_tolerance = _config.blue_h_offset_y_tolerance;

    this->red_x_offset_x_tolerance = _config.red_x_offset_x_tolerance;
    this->red_x_offset_y_tolerance = _config.red_x_offset_y_tolerance;

    this->qr_offset_x_tolerance = _config.qr_offset_x_tolerance;
    this->qr_offset_y_tolerance = _config.qr_offset_y_tolerance;

    this->tf_break_duration = _config.tf_break_duration;

    this->navigation_per_sustain_time = _config.navigation_per_sustain_time;
    this->judge_achieve_counter_threshold = _config.judge_achieve_counter_threshold;
}


double_t vwpp::DynamicRecfgInterface::getAltitudeTolerance() const
{
    return altitude_tolerance;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PXKp() const
{
    return pid_p_v2p_x_kp;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PXKi() const
{
    return pid_p_v2p_x_ki;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PXKd() const
{
    return pid_p_v2p_x_kd;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PYKp() const
{
    return pid_p_v2p_y_kp;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PYKi() const
{
    return pid_p_v2p_y_ki;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PYKd() const
{
    return pid_p_v2p_y_kd;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PZKp() const
{
    return pid_p_v2p_z_kp;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PZKi() const
{
    return pid_p_v2p_z_ki;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PZKd() const
{
    return pid_p_v2p_z_kd;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PXKp() const
{
    return pid_v_v2p_x_kp;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PXKi() const
{
    return pid_v_v2p_x_ki;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PXKd() const
{
    return pid_v_v2p_x_kd;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PYKp() const
{
    return pid_v_v2p_y_kp;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PYKi() const
{
    return pid_v_v2p_y_ki;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PYKd() const
{
    return pid_v_v2p_y_kd;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PZKp() const
{
    return pid_v_v2p_z_kp;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PZKi() const
{
    return pid_v_v2p_z_ki;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PZKd() const
{
    return pid_v_v2p_z_kd;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PYawKp() const
{
    return pid_p_v2p_yaw_kp;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PYawKi() const
{
    return pid_p_v2p_yaw_ki;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PYawKd() const
{
    return pid_p_v2p_yaw_kd;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PYawKp() const
{
    return pid_v_v2p_yaw_kp;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PYawKi() const
{
    return pid_v_v2p_yaw_ki;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PYawKd() const
{
    return pid_v_v2p_yaw_kd;
}


double_t vwpp::DynamicRecfgInterface::getTfBreakDuration() const
{
    return tf_break_duration;
}


double_t vwpp::DynamicRecfgInterface::getForwardVel() const
{
    return forward_vel;
}


double_t vwpp::DynamicRecfgInterface::getAltitudeWhenRedGate() const
{
    return altitude_when_red_gate;
}


double_t vwpp::DynamicRecfgInterface::getAltitudeWhenYellowGate() const
{
    return altitude_when_yellow_gate;
}


double_t vwpp::DynamicRecfgInterface::getAvoidanceForwardTime() const
{
    return avoidance_forward_time;
}


double_t vwpp::DynamicRecfgInterface::getNormalFlightAltitude() const
{
    return normal_flight_altitude;
}


double_t vwpp::DynamicRecfgInterface::getBlueHOffsetXTolerance() const
{
    return blue_h_offset_x_tolerance;
}


double_t vwpp::DynamicRecfgInterface::getBlueHOffsetYTolerance() const
{
    return blue_h_offset_y_tolerance;
}


double_t vwpp::DynamicRecfgInterface::getLandingAltitude() const
{
    return landing_altitude;
}


double_t vwpp::DynamicRecfgInterface::getRedXOffsetXTolerance() const
{
    return red_x_offset_x_tolerance;
}


double_t vwpp::DynamicRecfgInterface::getRedXOffsetYTolerance() const
{
    return red_x_offset_y_tolerance;
}


double_t vwpp::DynamicRecfgInterface::getRotateYawTolerance() const
{
    return rotate_yaw_tolerance;
}


double_t vwpp::DynamicRecfgInterface::getQrOffsetXTolerance() const
{
    return qr_offset_x_tolerance;
}


double_t vwpp::DynamicRecfgInterface::getQrOffsetYTolerance() const
{
    return qr_offset_y_tolerance;
}


bool vwpp::DynamicRecfgInterface::isPidPV2PXHasThreshold() const
{
    return pid_p_v2p_x_has_threshold;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PXThreshold() const
{
    return pid_p_v2p_x_threshold;
}


bool vwpp::DynamicRecfgInterface::isPidPV2PYHasThreshold() const
{
    return pid_p_v2p_y_has_threshold;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PYThreshold() const
{
    return pid_p_v2p_y_threshold;
}


bool vwpp::DynamicRecfgInterface::isPidPV2PZHasThreshold() const
{
    return pid_p_v2p_z_has_threshold;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PZThreshold() const
{
    return pid_p_v2p_z_threshold;
}


bool vwpp::DynamicRecfgInterface::isPidVV2PXHasThreshold() const
{
    return pid_v_v2p_x_has_threshold;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PXThreshold() const
{
    return pid_v_v2p_x_threshold;
}


bool vwpp::DynamicRecfgInterface::isPidVV2PYHasThreshold() const
{
    return pid_v_v2p_y_has_threshold;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PYThreshold() const
{
    return pid_v_v2p_y_threshold;
}


bool vwpp::DynamicRecfgInterface::isPidVV2PZHasThreshold() const
{
    return pid_v_v2p_z_has_threshold;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PZThreshold() const
{
    return pid_v_v2p_z_threshold;
}


bool vwpp::DynamicRecfgInterface::isPidPV2PYawHasThreshold() const
{
    return pid_p_v2p_yaw_has_threshold;
}


double_t vwpp::DynamicRecfgInterface::getPidPV2PYawThreshold() const
{
    return pid_p_v2p_yaw_threshold;
}


bool vwpp::DynamicRecfgInterface::isPidVV2PYawHasThreshold() const
{
    return pid_v_v2p_yaw_has_threshold;
}


double_t vwpp::DynamicRecfgInterface::getPidVV2PYawThreshold() const
{
    return pid_v_v2p_yaw_threshold;
}


int64_t vwpp::DynamicRecfgInterface::getJudgeAchieveCounterThreshold() const
{
    return judge_achieve_counter_threshold;
}


double_t vwpp::DynamicRecfgInterface::getNavigationPerSustainTime() const
{
    return navigation_per_sustain_time;
}


