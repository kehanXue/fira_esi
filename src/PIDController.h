//
// Created by kehan on 19-7-11.
//

#ifndef FIRA_ESI_PIDCONTROLLER_H_
#define FIRA_ESI_PIDCONTROLLER_H_


#include <cmath>
#include <cstdint>

namespace vwpp
{
    class PIDController
    {
    public:

        PIDController(double_t _kp, double_t _ki, double_t _kd, bool _has_threshold = false, double_t _threshold = 0);

        virtual ~PIDController();

        int8_t setTarget(double_t _target);

        double_t getTarget();

        int8_t update(double_t _cur);

        double_t output();


    private:

        bool has_threshold;
        double_t threshold;

        double_t kp;
        double_t ki;
        double_t kd;

        double_t target;

        double_t err;
        double_t ierr;
        double_t derr;

        double_t last_err;

    };

}


#endif //FIRA_ESI_PIDCONTROLLER_H_
