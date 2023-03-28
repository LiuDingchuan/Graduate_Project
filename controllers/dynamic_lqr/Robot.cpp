
/***
 * @CreatedTime   2022-04-23 19:57:04
 * @LastEditors   未定义
 * @LastEditTime  2022-04-25 15:18:25
 * @FilePath      \bishe\Robot.cpp
 */

#include "Robot.h"

MyRobot::MyRobot() : balance_angle(-0.0064)
{
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
/**
 * @brief:状态矩阵更新
 * @author: Dandelion
 * @Date: 2023-03-27 20:51:44
 * @param {LegClass} *leg
 * @param {PositionSensor} *encoder_L 这里是正视五连杆图形的left和right
 * @param {PositionSensor} *encoder_R 这里是正式五连杆图形的left和right
 * @param {PositionSensor} *encoder_Wheel
 * @param {float} dis
 * @param {float} dis_dot
 * @param {float} pitch
 * @return {*}
 */
void MyRobot::status_update(LegClass *leg,
                            PositionSensor *encoder_L, PositionSensor *encoder_R, PositionSensor *encoder_Wheel,
                            float dis, float dis_dot, float pitch, float pitch_dot, float dt,
                            float v_set)
{
    // 计算K矩阵数据，根据l0_now拟合得到
    static Matrix<float, 12, 4> K_coeff;
    K_coeff << -56.2657122197288, 110.955449839554, -158.932304773749, -13.2108628940147,
        124.121829405397, -115.781173481685, 26.3894368897019, 20.9758944326806,
        19.3122283093215, -28.1143514447985, -19.7975177682733, -0.665103815173968,
        27.7750019873454, -31.6416911517429, 15.3899867915492, 1.57561475576218,
        -33.2736580172453, 43.9057047808111, -21.0290308785676, -18.4179466672356,
        -101.744952999825, 161.913767522745, -98.5903891501989, 28.7533856041495,
        -18.0941404528499, 30.5738085289490, -27.0370386156508, -12.3401523210168,
        -47.5172565035432, 83.9660076041746, -55.9012364768479, 19.5060587585273,
        -160.872896199552, 256.008145196585, -155.885092637028, 45.4630944836172,
        210.441091502134, -277.684059389342, 132.999269320722, 116.485322563444,
        -11.3405133550703, 16.1543094417432, -8.97298973283048, 2.45814399283596,
        11.1745266351310, -14.5626418795488, 6.91332413437057, 3.36393511857054;
    leg->angle1 = -encoder_R->getValue() + 2 / 3 * PI;
    leg->angle4 = encoder_L->getValue() + 1 / 3 * PI;
    float angle0_before = leg->angle0;
    float pitch_before = pitch;
    leg->Zjie(leg->angle1, leg->angle4, pitch);
    leg->angle0_dot = (leg->angle0 - angle0_before) / dt;
    for (size_t col = 0; col < 6; col++)
    {
        /* code */
        for (size_t row = 0; row < 2; row++)
        {
            /* code */
            int num = col * 2 + row;
            leg->K(row, col) = K_coeff(num, 0) * pow(leg->L0_now, 3) +
                               K_coeff(num, 1) * pow(leg->L0_now, 2) +
                               K_coeff(num, 2) * leg->L0_now +
                               K_coeff(num, 3);
        }
    }
    // 状态更新
    leg->X << leg->angle0, leg->angle0_dot, dis, dis_dot, pitch, pitch_dot;
}

void MyRobot::run()
{
    static int last_key;

    static PID_Controller vertical_pid(9, 0, 1, 60);
    static PID_Controller velocity_pid(0.13, 0.003, 0.02, 1.0);
    static PID_Controller turn_pid(12, 0, 0.3, 0);
    static PID_Controller roll_pid(0.18, 0, 0.02, 0);

    pitch = imu->getRollPitchYaw()[1];
    pitch_dot = gyro->getValues()[2];
    roll = imu->getRollPitchYaw()[0];
    roll_dot = gyro->getValues()[0];
    yaw_get = imu->getRollPitchYaw()[2];
    yaw_dot = gyro->getValues()[1];
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

    disL = encoder_wheelL->getValue() * 0.05;
    disR = encoder_wheelR->getValue() * 0.05;

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

    disL_dot = (disL - disL_last) * 1000 / time_step;
    disR_dot = (disR - disR_last) * 1000 / time_step; // m/s
    velocity_set = Limit(velocity_set, 3, -3);
    velocity_out = velocity_pid.compute(velocity_set, (disL_dot + disR_dot) / 2);
    float pitch_set = velocity_out + balance_angle;
    disL_last = disL;
    disR_last = disR;

    vertical_out += vertical_pid.compute(pitch_set, pitch, pitch_dot);

    turn_out = turn_pid.compute(yaw_set, yaw, yaw_dot);

    roll_set = Limit(roll_set, 0.35, -0.35);
    float leg_out = 0;
    // leg_out = roll_pid.compute(roll_set, roll, roll_dot);

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

    // printf("pitch_set:%f, pitch:%f, disL_dot:%f, yaw:%f, L_y:%f, R_y:%f, leg_L.TL_now:%f, L_Torque:%f\n",
    //        pitch_set, pitch, disL_dot, yaw, leg_L.yc, leg_R.yc, leg_L.TL_now, L_Torque);

    printf("BackLeft:%f, FrontLeft:%f, LeftSpeed:%f, RightSpeed:%f\n",
           encoder_BL->getValue(), encoder_FL->getValue(), disL_dot, disR_dot);
    // ofstream outfile;
    // outfile.open("data2.dat", ios::trunc);
    // outfile << time << ' ' << pitch << ' ' << disL_dot << ' ' << robot_x << ' ' << L_Torque << endl;
    // outfile.close();
}
