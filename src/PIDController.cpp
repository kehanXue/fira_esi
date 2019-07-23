//
// Created by kehan on 19-7-11.
//

#include <cstdlib>
#include "PIDController.h"


vwpp::PIDController::PIDController(double_t _kp, double_t _ki, double_t _kd, bool _has_threshold, double_t _threshold) :
        has_threshold(_has_threshold),
        threshold(_threshold),
        kp(_kp),
        ki(_ki),
        kd(_kd),
        target(0.),
        err(0.),
        ierr(0.),
        derr(0.),
        last_err(0.)
{

}


vwpp::PIDController::~PIDController()
= default;


int8_t vwpp::PIDController::setTarget(double_t _target)
{
    target = _target;
    ierr = 0;

    return 0;
}


double_t vwpp::PIDController::getTarget()
{
    return this->target;
}


int8_t vwpp::PIDController::update(double_t _cur)
{
    err = target - _cur;
    ierr += err;
    derr = err - last_err;
    last_err = err;

    return 0;
}


double_t vwpp::PIDController::output()
{
    double_t ans = (kp * err + ki * ierr + kd * derr);

    if (has_threshold && (fabs(ans) > fabs(threshold)))
    {
        ans = (ans / fabs(ans)) * threshold;          // TODO Maybe not 1?
    }
    return ans;
}



