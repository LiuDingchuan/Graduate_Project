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

class Pid
{
private:
    /* data */
    float kp, ki, kd;
    float err_now, err_last, err_sum;
    float max_output;

    float a_Filter; // 滤波
    float err_lowout;

public:
    Pid(float _p, float _i, float _d, float _max_out);
    ~Pid();

    float compute(float set, float real, float angle_w);
    float compute(float set, float encoder);

    void clear();
    void update(float _p, float _i, float _d, float _max_out);
};

#endif
