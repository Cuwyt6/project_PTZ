//
// Created by lenovo on 24-11-21.
//
#include "main.h"

#ifndef PTZ_PID_H
#define PTZ_PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float i_max, float out_max);

    float calc(float ref, float fdb);
    float ref_, fdb_;
    //float kfilter_d_;
    float kp_, ki_, kd_;
    float i_max_, out_max_;
private:
    float output_;
    float err_, err_sum_, last_err_;
    float pout_, iout_, dout_;
};

#endif //PTZ_PID_H
