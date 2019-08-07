//
// Created by kehan on 19-7-23.
//

#ifndef PX4_MODULES_TEST_UTILS_H_
#define PX4_MODULES_TEST_UTILS_H_

#include <cmath>

double_t convertCurYaw2FabsYawThetaBetweenPI(double_t _target_yaw, double_t _cur_yaw);

double_t convertYaw2BetweenFabsPI(double_t _yaw);

double_t judgeTrackingLineVVelYScaleSigmoidFunction(double_t _cur_altitude);

#endif //PX4_MODULES_TEST_UTILS_H_
