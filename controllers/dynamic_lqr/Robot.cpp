
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

    BL_legmotor->enableTorqueFeedback(time_step), BR_legmotor->enableTorqueFeedback(time_step), FL_legmotor->enableTorqueFeedback(time_step), FR_legmotor->enableTorqueFeedback(time_step);
    L_Wheelmotor->enableTorqueFeedback(time_step), R_Wheelmotor->enableTorqueFeedback(time_step);
    // 调参
    velocity_set = 0;
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
 * @brief:
 * @author: Dandelion
 * @Date: 2023-04-01 00:10:38
 * @param {LegClass} *leg_sim
 * @param {LegClass} *leg_L
 * @param {LegClass} *leg_R
 * @param {float} pitch
 * @param {float} pitch_dot
 * @param {float} dt
 * @param {float} v_set
 * @return {*}
 */
void MyRobot::status_update(LegClass *leg_sim, LegClass *leg_L, LegClass *leg_R,
                            float pitch, float pitch_dot, float dt, float v_set)
{
    // 计算K矩阵数据，根据l0_now拟合得到
    static Matrix<float, 12, 4> K_coeff;
    static Matrix<float, 2, 1> u;
    static Matrix<float, 2, 1> Torque;
    K_coeff << -99.0512848225234, 161.375030514160, -156.389621410455, -10.0590062880495,
        63.9928715566516, -46.4720626070421, -6.92353815607583, 26.1259067572226,
        11.5441042370362, -14.8597922303923, -19.3625779434887, -0.299053916534314,
        19.3165893080792, -23.0045086815708, 10.9657841994598, 2.24279288946326,
        -32.6142934810401, 41.5073390771139, -19.0900634077401, -6.61000745160173,
        -81.5303433266699, 118.703731238366, -67.0637667898665, 17.9908215640236,
        -37.1793756145752, 50.2424056321152, -28.6464352051342, -8.20377989047164,
        -89.8958812316146, 132.980675257207, -76.7425492354827, 22.0219844113493,
        -40.7651716632755, 59.3518656190949, -33.5318833948964, 8.99541078200716,
        65.2285869732094, -83.0146781649621, 38.1801268189101, 13.2200149028407,
        -7.00458321609559, 9.95334225805404, -5.55431360604017, 1.53156019003240,
        11.6338020785579, -14.8445886620617, 6.88039672819413, 1.30268222126150;
    leg_sim->angle1 = (leg_L->angle4 + PI - leg_R->angle1) / 2;
    leg_sim->angle4 = (leg_R->angle4 + PI - leg_L->angle1) / 2;
    cout << "angle1: " << leg_sim->angle1 << " angle4: " << leg_sim->angle4 << endl;
    float angle0_before = leg_sim->angle0;
    leg_sim->Zjie(leg_sim->angle1, leg_sim->angle4, pitch);
    leg_sim->angle0_dot = (leg_sim->angle0 - angle0_before) / dt;
    printf("K: ");
    for (size_t col = 0; col < 6; col++)
    {
        /* code */
        for (size_t row = 0; row < 2; row++)
        {
            /* code */
            int num = col * 2 + row;
            leg_sim->K(row, col) = K_coeff(num, 0) * pow(leg_sim->L0_now, 3) +
                                   K_coeff(num, 1) * pow(leg_sim->L0_now, 2) +
                                   K_coeff(num, 2) * leg_sim->L0_now +
                                   K_coeff(num, 3);
            printf("%f ", leg_sim->K(row, col));
        }
    }
    printf("\n");
    // 状态更新
    leg_sim->dis_desire += v_set * dt;
    printf("Xd:%f\n", leg_sim->dis_desire);
    leg_sim->Xd << 0, 0, leg_sim->dis_desire, 0, 0, 0;
    leg_sim->X << leg_sim->angle0, leg_sim->angle0_dot, leg_sim->dis, leg_sim->dis_dot, pitch, pitch_dot;
    cout << leg_sim->angle0 << " " << angle0_before << " " << leg_sim->angle0_dot << " " << leg_sim->dis << " " << leg_sim->dis_dot << " " << pitch << " " << pitch_dot << endl;
    u = leg_sim->K * (leg_sim->Xd - leg_sim->X);
    leg_sim->TWheel_set = u(0, 0);
    leg_sim->Tp_set = u(1, 0);

    leg_L->TWheel_set = leg_sim->TWheel_set / 2.0;
    leg_R->TWheel_set = leg_sim->TWheel_set / 2.0;

    leg_L->Tp_set = leg_sim->Tp_set / 2.0;
    leg_R->Tp_set = leg_sim->Tp_set / 2.0;
    // 左腿VMC解算
    leg_L->L0_dot = (leg_L->L0_now - leg_L->L0_last) / dt;
    leg_L->L0_last = leg_L->L0_now;
    leg_L->F_set += leg_L->supportF_pid.compute(leg_L->L0_set, 0, leg_L->L0_now, leg_L->L0_dot, dt);
    cout << "F_set: " << leg_L->F_set << endl;
    Torque = leg_L->VMC(leg_L->F_set, leg_L->Tp_set);
    leg_L->TR_set = -Torque(0, 0);
    leg_L->TL_set = -Torque(1, 0);
    // 右腿VMC解算
    leg_R->L0_dot = (leg_R->L0_now - leg_R->L0_last) / dt;
    leg_R->L0_last = leg_R->L0_now;
    leg_R->F_set += leg_R->supportF_pid.compute(leg_R->L0_set, 0, leg_R->L0_now, leg_R->L0_dot, dt);
    cout << "F_set: " << leg_R->F_set << endl;
    Torque = leg_R->VMC(leg_R->F_set, leg_R->Tp_set);
    leg_R->TR_set = -Torque(0, 0);
    leg_R->TL_set = -Torque(1, 0);
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
    // 获取当前机器人状态信息
    leg_L.dis = encoder_wheelL->getValue() * 0.05;
    leg_R.dis = encoder_wheelR->getValue() * 0.05;
    leg_L.dis_dot = (leg_L.dis - leg_R.dis_last) * 1000.f / time_step;
    leg_R.dis_dot = (leg_R.dis - leg_R.dis_last) * 1000.f / time_step;
    leg_L.dis_last = leg_L.dis;
    leg_R.dis_last = leg_R.dis;
    leg_L.TL_now = BL_legmotor->getTorqueFeedback();
    leg_L.TR_now = FL_legmotor->getTorqueFeedback();
    leg_R.TL_now = BR_legmotor->getTorqueFeedback();
    leg_R.TR_now = FR_legmotor->getTorqueFeedback();
    leg_L.TWheel_now = L_Wheelmotor->getTorqueFeedback();
    leg_R.TWheel_now = L_Wheelmotor->getTorqueFeedback();
    // 角度更新
    leg_L.angle1 = 2.0 / 3.0 * PI - encoder_FL->getValue();
    leg_L.angle4 = 1.0 / 3.0 * PI + encoder_BL->getValue();
    leg_L.Zjie(leg_L.angle1, leg_L.angle4, pitch);
    leg_R.angle1 = 2.0 / 3.0 * PI - encoder_FR->getValue();
    leg_R.angle4 = 1.0 / 3.0 * PI + encoder_BR->getValue();
    leg_R.Zjie(leg_R.angle1, leg_R.angle4, pitch);

    // 时序更新
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
            leg_L.yc += 0.002f;
            leg_R.yc += 0.002f;
            break;
        case 'S':
            leg_L.yc -= 0.002f;
            leg_R.yc -= 0.002f;
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

    velocity_set = Limit(velocity_set, 3, -3);
    // velocity_out = velocity_pid.compute(velocity_set, (leg_L.dis_dot + leg_R.dis_dot) / 2);
    // float pitch_set = velocity_out + balance_angle;

    // vertical_out += vertical_pid.compute(pitch_set, pitch, pitch_dot);

    // turn_out = turn_pid.compute(yaw_set, yaw, yaw_dot);

    // roll_set = Limit(roll_set, 0.35, -0.35);
    // float leg_out = 0;
    // // leg_out = roll_pid.compute(roll_set, roll, roll_dot);

    // leg_L.yc = Limit(leg_L.yc + leg_out, 0.37, 0.120);
    // leg_R.yc = Limit(leg_R.yc - leg_out, 0.37, 0.120);

    // leg_L.Njie(leg_L.xc, leg_L.yc);
    // leg_R.Njie(leg_R.xc, leg_R.yc);
    cout << "左腿更新：" << endl;
    status_update(&leg_simplified, &leg_L, &leg_R, pitch, pitch_dot, time_step / 1000.f, velocity_set);
    // cout << "右腿更新：" << endl;
    // status_update(&leg_R, encoder_FR, encoder_BR, encoder_wheelR, pitch, pitch_dot, time_step / 1000.f, velocity_set);

    cout << "L_TL " << leg_L.TL_now << " L_TR " << leg_L.TR_now << " R_TL " << leg_R.TL_now << " R_TR " << leg_R.TR_now << " L_Wheel" << leg_L.TWheel_now << "R_Wheel" << leg_R.TWheel_now << endl;
    // BL_legmotor->setPosition(-(leg_L.angle1 - PI * 2 / 3)); // 懒得管正方向怎么看了，就这样吧，反正顺负逆正
    // FL_legmotor->setPosition(leg_L.angle4 - PI / 3);
    // BR_legmotor->setPosition(-(leg_R.angle1 - PI * 2 / 3));
    // FR_legmotor->setPosition(leg_R.angle4 - PI / 3);
    // L_Wheelmotor->setVelocity(Limit(vertical_out - turn_out, 60, -60));
    // R_Wheelmotor->setVelocity(Limit(vertical_out + turn_out, 60, -60));
    // BL_legmotor->setTorque(leg_L.TR_set);
    // FL_legmotor->setTorque(leg_L.TL_set);
    // BR_legmotor->setTorque(leg_R.TL_set);
    // FL_legmotor->setTorque(leg_R.TR_set);
    // // // // L_Wheelmotor->setPosition(0);
    // // // // R_Wheelmotor->setPosition(0);
    L_Wheelmotor->setTorque(leg_L.TWheel_set);
    L_Wheelmotor->setTorque(leg_R.TWheel_set);

    printf("dt:%d, BackLeft:%f, FrontLeft:%f, BackRight:%f, FrontRight:%f, WheelL:%f, WheelR:%f, L0_set:%f, L0_now:%f, angle2:%f, angle3:%f\n",
           time_step, leg_L.TR_set, leg_L.TL_set, leg_R.TL_set, leg_R.TR_set, leg_L.TWheel_set, leg_R.TWheel_set, leg_L.L0_set, leg_L.L0_now, leg_L.angle2, leg_L.angle3);
    // ofstream outfile;
    // outfile.open("data2.dat", ios::trunc);
    // outfile << time << ' ' << pitch << ' ' << disL_dot << ' ' << robot_x << ' ' << L_Torque << endl;
    // outfile.close();
}
