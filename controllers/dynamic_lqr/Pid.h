/***
 * @CreatedTime   2021-12-16 16:27:27
 * @LastEditors   未定义
 * @LastEditTime  2021-12-16 16:28:37
 * @FilePath      \TwoWheelCpp\pid.h
 */
#ifndef PID_H
#define PID_H

#include "MyDefine.h"
#include <iostream>

using namespace std;

class PID_Controller
{
private:
    /* data */
    float kp, ki, kd;
    float err_now, err_last, err_sum, err_dot;
    float output, max_output, max_sum;

    float a_Filter; // 滤波
    float err_lowout;

public:
    PID_Controller() : kp(0), ki(0), kd(0), err_sum(0), output(0), max_sum(0), max_output(0){};
    PID_Controller(float _p, float _i, float _d, float _max_out);
    ~PID_Controller();

    float compute(float set, float real, float angle_w);
    float compute(float set, float encoder);
    float compute(const float target, const float d_target, const float input, const float d_input, const float dt);

    void clear();
    void update(float _p, float _i, float _d, float _max_out);
};

#endif