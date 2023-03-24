/***
 * @CreatedTime   2022-04-23 19:57:04
 * @LastEditors   未定义
 * @LastEditTime  2022-04-24 20:16:23
 * @FilePath      \bishe\Pid.cpp
 */
/***
 * @CreatedTime   2022-04-23 19:57:04
 * @LastEditors   未定义
 * @LastEditTime  2022-04-23 20:07:20
 * @FilePath      \bishe\Pid.cpp
 */

#include "Pid.h"

Pid::Pid(float _p, float _i, float _d, float _max_out)
    : kp(_p), ki(_i), kd(_d), max_output(_max_out)
{
    if (0 == _max_out)
        max_output = 99999;

    a_Filter = 0.7;

    clear();
}

Pid::~Pid()
{
}

void Pid::clear()
{
    err_sum = 0, err_now = 0, err_last = 0, err_lowout = 0;
}

void Pid::update(float _p, float _i, float _d, float _max_out)
{
    kp = _p, ki = _i, kd = _d, max_output = _max_out;
}

float Pid::compute(float set, float real, float angle_w)
{
    err_now = set - real;
    float out = kp * err_now - kd * angle_w;
    out = Limit(out, max_output, -max_output);
    return out;
}

float Pid::compute(float set, float encoder)
{
    err_now = set - encoder;
    err_lowout = (1 - a_Filter) * err_now + a_Filter * err_last;
    err_sum += err_lowout;
    err_last = err_lowout;
    err_sum = Limit(err_sum, 15, -15);
    // cout << "err_sum:" << err_sum << endl;

    if (ABS(set) > 0.01f)
    {
        ki = 0;
        err_sum = 0;
    }
    float out = kp * err_lowout + ki * err_sum + kd * (err_lowout - err_last);
    out = Limit(out, max_output, -max_output);
    return -out;
}
