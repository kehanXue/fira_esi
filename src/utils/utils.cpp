//
// Created by kehan on 19-7-23.
//

#include "utils.h"


double_t convertCurYaw2FabsYawThetaBetweenPI(double_t _target_yaw, double_t _cur_yaw)
{
    double_t yaw_theta = _target_yaw - _cur_yaw;

    while (yaw_theta > M_PI)
    {
        yaw_theta -= 2 * M_PI;
    }
    while (yaw_theta < -M_PI)
    {
        yaw_theta += 2 * M_PI;
    }

    return (_target_yaw - yaw_theta);
}


double_t judgeTrackingLineVVelYScaleSigmoidFunction(double_t _cur_altitude)
{
    return ((1 / (1 + exp(-(_cur_altitude * _cur_altitude) * 5.3))) - 0.5) * 2;
}

