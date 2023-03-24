/***
 * @CreatedTime   2022-04-23 19:57:04
 * @LastEditors   未定义
 * @LastEditTime  2022-04-25 15:29:11
 * @FilePath      \bishe\Robot.h
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <fstream>
#include "MyDefine.h"
#include "Pid.h"
#include <math.h>
#include <Eigen>

using namespace Eigen;
using namespace webots;
using namespace std;

#define NMOTORS 6
#define Link1 180
#define Link2 200
#define Link3 200
#define Link4 180
#define Link5 120

class LegClass
{
public:
    // 长度
    float angle0, angle1, angle2, angle3, angle4;
    float L0_now;
    float L0_set;
    // 坐标
    float xa, ya;
    float xb, yb;
    float xc, yc;
    float xd, yd;
    float xe, ye;
    // 力与力矩
    float TL_now, TR_now, TWheel_now; // 左腿看左视图，右腿看右视图
    float TL_set, TR_set, TWheel_set;
    float F_set;  // 根据腿长PD控制得到， 初值为上层机构重力
    float Tp_set; // 根据状态反馈矩阵得到
};

class MyRobot : public Robot
{
private:
    /* data */
    u8 time_step;
    float time;

    Camera *camera;
    Gyro *gyro;
    InertialUnit *imu;
    GPS *gps;
    PositionSensor *encoder_L, *encoder_R;
    Motor *BL_legmotor, *BR_legmotor, *FL_legmotor, *FR_legmotor, *L_Wheelmotor, *R_Wheelmotor;
    Keyboard *mkeyboard;

    LegClass leg_L, leg_R;

    Matrix<float, 2, 6> K_L, K_R; // 反馈矩阵
    Matrix<float, 6, 1> X_L, X_R; // 状态矩阵，[theta, theta_dot, x, x_dot, phi, phi_dot]

    float velocity_set, yaw_set, roll_set;
    float velocity_out, vertical_out, turn_out, leg_out;
    float encoderL_last, encoderR_last, encoderL_now, encoderR_now;
    float disL, disR;
    float disL_dot, disR_dot;
    float pitch, roll, yaw, pitch_w, roll_w, yaw_w;
    float yaw_get, yaw_get_last;

    u8 stop_flag;
    void Zjie(LegStruct *leg);
    void Njie(LegStruct *leg);
    Matrix<float, 2, 1> VMC();

public:
    MyRobot();
    ~MyRobot();

    float balance_angle;

    static MyRobot *get()
    {
        static MyRobot robot;
        return &robot;
    }

    u8 jump(float t_clk, float *leg_L, float *leg_R);
    void update();
    void MyStep();
    void Wait(int ms);
    void run();
    double getVelNow() { return gps->getSpeed(); };
    double getVelSet() { return velocity_set; };
};

#endif
