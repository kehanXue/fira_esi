//
// Created by kehan on 19-7-11.
//

#include "Action.h"
#include "PIDController.h"

vwpp::Velocity2D
vwpp::Action::trackingLine(double_t _cur_line_y, double_t _cur_yaw, vwpp::Direction _direction,
                           double_t _forward_vel)
{
    // TODO Get param from ros param serrver.
    static PIDController pid_controller_body_y(1.0, 0.0, 1.0);
    static PIDController pid_controller_body_yaw(1.0, 0.0, 1.0);

    pid_controller_body_y.setTarget(0);
    pid_controller_body_y.update(_cur_line_y);

    pid_controller_body_y.setTarget(0);
    pid_controller_body_yaw.update(_cur_yaw);

    Velocity2D velocity_2d{};
    velocity_2d.x = _forward_vel;
    velocity_2d.y = pid_controller_body_y.output();
    velocity_2d.yaw = pid_controller_body_yaw.output();


    return velocity_2d;
}

vwpp::VelocityZ vwpp::Action::adjustAltitude(double_t _target_altitude, double_t _cur_altitude)
{
    static PIDController pid_controller_local_z(1.0, 0.0, 1.0);
    pid_controller_local_z.setTarget(_target_altitude);
    pid_controller_local_z.update(_cur_altitude);

    return pid_controller_local_z.output();
}

vwpp::Velocity2D vwpp::Action::hovering(double_t _cur_x, double_t _cur_y, double_t _cur_yaw)
{
    return vwpp::Velocity2D();
}

int8_t vwpp::Action::openClaw()
{
    return 0;
}


