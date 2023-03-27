
/***
 * @CreatedTime   2022-04-23 19:57:04
 * @LastEditors   未定义
 * @LastEditTime  2022-04-25 15:18:25
 * @FilePath      \bishe\Robot.cpp
 */

#include "Robot.h"

MyRobot::MyRobot() : balance_angle(-0.0064)
{
    leg_L.xc = 60, leg_L.yc = 288.17, leg_R.xc = 60, leg_R.yc = 288.17;
    leg_L.angle1 = 0, leg_L.angle2 = 0, leg_R.angle1 = 0, leg_R.angle2 = 0;

    time_step = getBasicTimeStep();
    mkeyboard = getKeyboard(), mkeyboard->enable(time_step);
    camera = getCamera("camera"), camera->enable(time_step);
    gyro = getGyro("gyro"), gyro->enable(time_step);
    imu = getInertialUnit("imu"), imu->enable(time_step);
    gps = getGPS("gps"), gps->enable(time_step);
    encoder_wheelL = getPositionSensor("encoder_wheelL"), encoder_wheelL->enable(time_step);
    encoder_wheelR = getPositionSensor("encoder_wheelR"), encoder_wheelR->enable(time_step);
    encoder_BL = getPositionSensor("encoder_BL"), encoder_BL->enable(time_step);
    encoder_BR = getPositionSensor("encoder_BR"), encoder_BR->enable(time_step);
    encoder_FL = getPositionSensor("encoder_FL"), encoder_FL->enable(time_step);
    encoder_FR = getPositionSensor("encoder_FR"), encoder_FR->enable(time_step);
    L_Wheelmotor = getMotor("L_Motor"), BL_legmotor = getMotor("BL_Motor"), FL_legmotor = getMotor("FL_Motor");
    R_Wheelmotor = getMotor("R_Motor"), BR_legmotor = getMotor("BR_Motor"), FR_legmotor = getMotor("FR_Motor");
    L_Wheelmotor->setVelocity(0), R_Wheelmotor->setVelocity(0);
    L_Wheelmotor->setPosition(INFINITY), R_Wheelmotor->setPosition(INFINITY);
    BL_legmotor->setPosition(0), BR_legmotor->setPosition(0), FL_legmotor->setPosition(0), FR_legmotor->setPosition(0);

    BL_legmotor->enableTorqueFeedback(time_step), BR_legmotor->enableTorqueFeedback(time_step), FL_legmotor->enableTorqueFeedback(time_step), FL_legmotor->enableTorqueFeedback(time_step);
    L_Wheelmotor->enableTorqueFeedback(time_step), R_Wheelmotor->enableTorqueFeedback(time_step);
    // 调参
    turn_pid.update(1.0, 0.0, 0.01, 0);
    split_pid.update(1.0, 0.0, 0.01, 0);
}

MyRobot::~MyRobot()
{
}

void MyRobot::MyStep()
{
    if (step(time_step) == -1)
        exit(EXIT_SUCCESS);
}
/**
 * @brief: 毫秒级延时
 * @author: Dandelion
 * @Date: 2023-03-27 16:35:24
 * @param {int} ms
 * @return {*}
 */
void MyRobot::Wait(int ms)
{
    float start_time = getTime();
    float s = ms / 1000.0;
    while (s + start_time >= getTime())
        MyStep();
}

void MyRobot::status_update(LegClass *leg,
                            PositionSensor *encoder_L, PositionSensor *encoder_R, PositionSensor *encoder_Wheel,
                            float dis, float dis_dot)
{
    leg->angle1 = encoder_R->getValue();
    leg->angle4 = encoder_L->getValue();
}

void MyRobot::run()
{
    static int last_key;

    static PID_Controller vertical_pid(9, 0, 1, 60);
    static PID_Controller velocity_pid(0.13, 0.003, 0.02, 1.0);
    static PID_Controller turn_pid(12, 0, 0.3, 0);
    static PID_Controller roll_pid(0.18, 0, 0.02, 0);

    pitch = imu->getRollPitchYaw()[1];
    pitch_w = gyro->getValues()[2];
    roll = imu->getRollPitchYaw()[0];
    roll_w = gyro->getValues()[0];
    yaw_get = imu->getRollPitchYaw()[2];
    yaw_w = gyro->getValues()[1];
    float robot_x = gps->getValues()[0];

    if (yaw_get - yaw_get_last > 1.5 * PI)
        yaw += yaw_get - yaw_get_last - 2 * PI;
    else if (yaw_get - yaw_get_last < -1.5 * PI)
        yaw += yaw_get - yaw_get_last + 2 * PI;
    else
        yaw += yaw_get - yaw_get_last;

    yaw_get_last = yaw_get;
    if (time == 0)
        yaw_set = yaw;

    encoderL_now = encoder_wheelL->getValue();
    encoderR_now = encoder_wheelR->getValue();

    time = getTime();

    int key = mkeyboard->getKey();
    while (key > 0)
    {
        printf("key: %d\n", key);

        Keyboard kboard;
        switch (key)
        {
        case kboard.UP:
            velocity_set += 0.02f;
            break;
        case kboard.DOWN:
            velocity_set -= 0.02f;
            break;
        case kboard.RIGHT:
            if (ABS(velocity_set) > 0.1)
                yaw_set = yaw - 0.1f;
            else
                yaw_set = yaw - 0.2f;
            break;
        case kboard.LEFT:
            if (ABS(velocity_set) > 0.1)
                yaw_set = yaw + 0.1f;
            else
                yaw_set = yaw + 0.2f;
            break;
        case kboard.END:
            velocity_set = 0;
            break;
        case 'W':
            leg_L.yc += 2.f;
            leg_R.yc += 2.f;
            break;
        case 'S':
            leg_L.yc -= 2.f;
            leg_R.yc -= 2.f;
            break;
        case 'A':
            roll_set -= 0.02f;
            break;
        case 'D':
            roll_set += 0.02f;
            break;
        case ' ':
            if (last_key != key)
                stop_flag = True;
            break;
        }
        last_key = key;
        key = mkeyboard->getKey();
    }

    float L_speed = (encoderL_now - encoderL_last) * 50 / time_step, R_speed = (encoderR_now - encoderR_last) * 50 / time_step; // m/s
    velocity_set = Limit(velocity_set, 3, -3);
    velocity_out = velocity_pid.compute(velocity_set, (L_speed + R_speed) / 2);
    float pitch_set = velocity_out + balance_angle;
    encoderL_last = encoderL_now;
    encoderR_last = encoderR_now;

    vertical_out += vertical_pid.compute(pitch_set, pitch, pitch_w);

    turn_out = turn_pid.compute(yaw_set, yaw, yaw_w);

    roll_set = Limit(roll_set, 0.35, -0.35);
    float leg_out = 0;
    // leg_out = roll_pid.compute(roll_set, roll, roll_w);

    leg_L.yc = Limit(leg_L.yc + leg_out, 370, 120);
    leg_R.yc = Limit(leg_R.yc - leg_out, 370, 120);

    leg_L.Njie(leg_L.xc, leg_L.yc);
    leg_R.Njie(leg_R.xc, leg_R.yc);

    leg_L.TL_now = BL_legmotor->getTorqueFeedback();
    leg_L.TR_now = FL_legmotor->getTorqueFeedback();
    leg_R.TL_now = BR_legmotor->getTorqueFeedback();
    leg_R.TR_now = FR_legmotor->getTorqueFeedback();
    float L_Torque = L_Wheelmotor->getTorqueFeedback();
    float R_Torque = R_Wheelmotor->getTorqueFeedback();

    BL_legmotor->setPosition(-(leg_L.angle1 - PI * 2 / 3)); // 懒得管正方向怎么看了，就这样吧，反正顺负逆正
    FL_legmotor->setPosition(leg_L.angle4 - PI / 3);
    BR_legmotor->setPosition(-(leg_R.angle1 - PI * 2 / 3));
    FR_legmotor->setPosition(leg_R.angle4 - PI / 3);

    L_Wheelmotor->setVelocity(Limit(vertical_out - turn_out, 60, -60));
    R_Wheelmotor->setVelocity(Limit(vertical_out + turn_out, 60, -60));

    // printf("pitch_set:%f, pitch:%f, L_speed:%f, yaw:%f, L_y:%f, R_y:%f, leg_L.TL_now:%f, L_Torque:%f\n",
    //        pitch_set, pitch, L_speed, yaw, leg_L.yc, leg_R.yc, leg_L.TL_now, L_Torque);

    printf("BackLeft:%f, FrontLeft:%f\n",
           encoder_BL->getValue(), encoder_FL->getValue());
    // ofstream outfile;
    // outfile.open("data2.dat", ios::trunc);
    // outfile << time << ' ' << pitch << ' ' << L_speed << ' ' << robot_x << ' ' << L_Torque << endl;
    // outfile.close();
}
