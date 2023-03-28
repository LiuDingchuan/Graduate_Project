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
#include <math.h>
#include <Eigen>
#include "MyDefine.h"
#include "Pid.h"
#include "Leg.h"

using namespace Eigen;
using namespace webots;
using namespace std;

#define NMOTORS 6
#define Link1 180
#define Link2 200
#define Link3 200
#define Link4 180
#define Link5 120

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
    PositionSensor *encoder_wheelL, *encoder_wheelR, *encoder_BL, *encoder_BR, *encoder_FL, *encoder_FR;
    Motor *BL_legmotor, *BR_legmotor, *FL_legmotor, *FR_legmotor, *L_Wheelmotor, *R_Wheelmotor;
    Keyboard *mkeyboard;

    LegClass leg_L, leg_R;
    PID_Controller turn_pid;
    PID_Controller split_pid;

    float velocity_set, yaw_set, roll_set;
    float velocity_out, vertical_out, turn_out, leg_out;
    float disL_last, disR_last;
    float disL, disR;
    float disL_dot, disR_dot;
    float pitch, roll, yaw, pitch_w, roll_w, yaw_w;
    float yaw_get, yaw_get_last;

    u8 stop_flag;

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
    void status_update(LegClass *leg,
                       PositionSensor *encoder_L, PositionSensor *encoder_R, PositionSensor *encoder_Wheel,
                       float dis, float dis_dot, float pitch, float pitch_w, float dt,
                       float v_set);
    void MyStep();
    void Wait(int ms);
    void run();
    double getVelNow() { return gps->getSpeed(); };
    double getVelSet() { return velocity_set; };
};

#endif
