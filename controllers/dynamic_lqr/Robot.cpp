
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
    encoder_L = getPositionSensor("encoder_L"),
    encoder_L->enable(time_step);
    encoder_R = getPositionSensor("encoder_R"), encoder_R->enable(time_step);
    L_Wheelmotor = getMotor("L_Motor"), BL_legmotor = getMotor("BL_Motor"), FL_legmotor = getMotor("FL_Motor");
    R_Wheelmotor = getMotor("R_Motor"), BR_legmotor = getMotor("BR_Motor"), FR_legmotor = getMotor("FR_Motor");
    L_Wheelmotor->setVelocity(0), R_Wheelmotor->setVelocity(0);
    L_Wheelmotor->setPosition(INFINITY), R_Wheelmotor->setPosition(INFINITY);
    BL_legmotor->setPosition(0), BR_legmotor->setPosition(0), FL_legmotor->setPosition(0), FR_legmotor->setPosition(0);

    BL_legmotor->enableTorqueFeedback(time_step), BR_legmotor->enableTorqueFeedback(time_step), FL_legmotor->enableTorqueFeedback(time_step), FL_legmotor->enableTorqueFeedback(time_step);
    L_Wheelmotor->enableTorqueFeedback(time_step), R_Wheelmotor->enableTorqueFeedback(time_step);
}

MyRobot::~MyRobot()
{
}

void MyRobot::MyStep()
{
    if (step(time_step) == -1)
        exit(EXIT_SUCCESS);
}

void MyRobot::Wait(int ms)
{
    float start_time = getTime();
    float s = ms / 1000.0;
    while (s + start_time >= getTime())
        MyStep();
}
/**
 * @brief: 逆解
 * @author: Dandelion
 * @Date: 2023-03-24 16:07:44
 * @param {LegStruct} *leg
 * @return {*}
 */
void MyRobot::Njie(LegStruct *leg)
{
    float m, n, b, x1, y1;
    float A, B, C;

    A = 2 * Link1 * leg->yc;
    B = 2 * Link1 * leg->xc;
    C = Link2 * Link2 - Link1 * Link1 - leg->xc * leg->xc - leg->yc * leg->yc;
    leg->angle1 = 2 * atan((A + sqrt(A * A + B * B - C * C)) / (B - C));
    if (leg->angle1 < 0)
        leg->angle1 += 2 * PI;

    // nije_5(&leg->angle1, (void *)0, leg->x, leg->y, Link1, Link6, Link3, Link4, Link5); //利用L1,L6计算c1;
    m = Link1 * cos(leg->angle1);
    n = Link1 * sin(leg->angle1);
    b = 0;
    // x1 = Link2 / Link6 * ((leg->x - m) * cos(b) - (leg->y - n) * sin(b)) + m;
    // y1 = Link2 / Link6 * ((leg->x - m) * sin(b) + (leg->y - n) * cos(b)) + n; //得到闭链五杆端点的坐标
    x1 = ((leg->xc - m) * cos(b) - (leg->yc - n) * sin(b)) + m;
    y1 = ((leg->xc - m) * sin(b) + (leg->yc - n) * cos(b)) + n; // 得到闭链五杆端点的坐标

    A = 2 * y1 * Link4;
    B = 2 * Link4 * (x1 - Link5);
    // c = Link3 * Link3 + 2 * Link5 * x1 - Link4 * Link4 - Link5 * Link5 - x1 * x1 - y1 * y1;
    C = Link3 * Link3 + 2 * Link5 * x1 - Link4 * Link4 - Link5 * Link5 - x1 * x1 - y1 * y1;
    leg->angle2 = 2 * atan((A - sqrt(A * A + B * B - C * C)) / (B - C));
    // nije_5((void *)0, &leg->angle2, x1, y1, Link1, Link2, Link3, Link4, Link5);        //计算c4 ,

    leg->angle1 -= PI * 2 / 3; // 计算坐标系与运动起始位置的差值
    leg->angle2 -= PI / 3;
}
/**
 * @brief: 运动学正解
 * @author: Dandelion
 * @Date: 2023-03-24 16:09:18
 * @param {LegStruct} *leg
 * @return {*}
 */
void MyRobot::Zjie(LegStruct *leg)
{
}

void MyRobot::run()
{
    static int last_key;

    static Pid vertical_pid(9, 0, 1, 60);
    static Pid velocity_pid(0.13, 0.003, 0.02, 1.0);
    static Pid turn_pid(12, 0, 0.3, 0);
    static Pid roll_pid(0.18, 0, 0.02, 0);

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

    encoderL_now = encoder_L->getValue();
    encoderR_now = encoder_R->getValue();

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

    Njie(&leg_L);
    Njie(&leg_R);

    leg_L.TL_now = BL_legmotor->getTorqueFeedback();
    leg_L.TorqueR = FL_legmotor->getTorqueFeedback();
    leg_R.TL_now = BR_legmotor->getTorqueFeedback();
    leg_R.TorqueR = FR_legmotor->getTorqueFeedback();
    float L_Torque = L_Wheelmotor->getTorqueFeedback();
    float R_Torque = R_Wheelmotor->getTorqueFeedback();

    BL_legmotor->setPosition(-leg_L.angle1);
    FL_legmotor->setPosition(leg_L.angle2);
    BR_legmotor->setPosition(-leg_R.angle1);
    FR_legmotor->setPosition(leg_R.angle2);

    L_Wheelmotor->setVelocity(Limit(vertical_out - turn_out, 60, -60));
    R_Wheelmotor->setVelocity(Limit(vertical_out + turn_out, 60, -60));

    printf("pitch_set:%f, pitch:%f, L_speed:%f, yaw:%f, L_y:%f, R_y:%f, leg_L.TL_now:%f, L_Torque:%f\n",
           pitch_set, pitch, L_speed, yaw, leg_L.y, leg_R.y, leg_L.TL_now, L_Torque);

    // ofstream outfile;
    // outfile.open("data2.dat", ios::trunc);
    // outfile << time << ' ' << pitch << ' ' << L_speed << ' ' << robot_x << ' ' << L_Torque << endl;
    // outfile.close();
}
