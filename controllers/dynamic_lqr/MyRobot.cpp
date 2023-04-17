

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
    // 参数初始化
    roll_set = 0, yaw.set = 0, velocity_set = 0;
    // 调参
    turn_pid.update(1.0, 0.0, 0.01, 0);
    split_pid.update(5.0, 0.0, 0.5, 0);
    roll_pid.update(50, 0.0, 0.5, 5);

    K_coeff << -64.8514955814788, 88.8287457455019, -58.0468300989671, -38.2739635886375,
        -225.794552414589, 308.707128464600, -155.036203000140, 30.8796260774293,
        13.5827287726877, -14.3490848814024, -5.24453913450503, -4.20911711372000,
        -21.1684728626557, 29.6703609535489, -15.4748734160391, 3.36279562153281,
        -3.07593168259110, 3.65198160363263, -1.50569510801103, -4.25237536412151,
        -29.0380727699917, 39.2060272063285, -19.3812304676001, 3.76888667869286,
        14.7811357012004, -18.9956679604013, 8.57218322858485, -8.25462347236922,
        -54.8372318769331, 71.8244716205884, -34.2089959588822, 6.34202356923289,
        -229.566122017227, 309.950859926968, -153.222080329443, 29.7956653686764,
        97.2695004074363, -115.485798373412, 47.6142600208101, 134.471916167180,
        -10.7102282846357, 14.6545751217460, -7.55471115824260, 1.66743331979285,
        3.41776165925051, -4.10246545269155, 1.72151782380153, 3.97764994859375;
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
                            const DataStructure pitch, const DataStructure roll, const DataStructure yaw,
                            const float dt, const float v_set)
{
    leg_L->F_set = 12 / 2 * 9.81;
    leg_R->F_set = 12 / 2 * 9.81;
    // 获取当前机器人状态信息
    leg_L->dis.now = encoder_wheelL->getValue() * 0.05;
    leg_R->dis.now = encoder_wheelR->getValue() * 0.05;
    leg_sim->dis.now = (leg_L->dis.now + leg_R->dis.now) / 2.0;

    leg_L->dis.dot = (leg_L->dis.now - leg_R->dis.last) * 1000.f / time_step;
    leg_R->dis.dot = (leg_R->dis.now - leg_R->dis.last) * 1000.f / time_step;
    leg_sim->dis.dot = (leg_L->dis.dot + leg_R->dis.dot) / 2.0;

    leg_L->dis.last = leg_L->dis.now;
    leg_R->dis.last = leg_R->dis.now;
    leg_sim->dis.last = (leg_L->dis.last + leg_R->dis.last) / 2;

    leg_L->TL_now = BL_legmotor->getTorqueFeedback();
    leg_L->TR_now = FL_legmotor->getTorqueFeedback();
    leg_R->TL_now = BR_legmotor->getTorqueFeedback();
    leg_R->TR_now = FR_legmotor->getTorqueFeedback();
    leg_L->TWheel_now = L_Wheelmotor->getTorqueFeedback();
    leg_R->TWheel_now = L_Wheelmotor->getTorqueFeedback();
    // 角度更新，统一从右视图看吧
    leg_L->angle1 = 2.0 / 3.0 * PI - encoder_FL->getValue();
    leg_L->angle4 = 1.0 / 3.0 * PI + encoder_BL->getValue();
    leg_L->Zjie(leg_L->angle1, leg_L->angle4, pitch.now);

    leg_R->angle1 = 2.0 / 3.0 * PI - encoder_FR->getValue();
    leg_R->angle4 = 1.0 / 3.0 * PI + encoder_BR->getValue();
    leg_R->Zjie(leg_R->angle1, leg_R->angle4, pitch.now);
    // 计算K矩阵数据，根据L0.now拟合得到
    static Matrix<float, 2, 1> u;
    static Matrix<float, 2, 1> Torque_L, Torque_R;
    leg_sim->angle1 = (leg_L->angle1 + leg_R->angle1) / 2;
    leg_sim->angle4 = (leg_L->angle4 + leg_R->angle4) / 2;
    leg_sim->Zjie(leg_sim->angle1, leg_sim->angle4, pitch.now);
    leg_sim->angle0.dot = (leg_sim->angle0.now - leg_sim->angle0.last) / dt;
    // 限幅
    // pitch_dot = Deadzone(pitch_dot, 0.01);
    // leg_sim->dis.dot = Deadzone(leg_sim->dis.dot, 0.01);
    // leg_sim->angle0_dot = Deadzone(leg_sim->angle0_dot, 0.1);
    for (size_t col = 0; col < 6; col++)
    {
        /* code */
        for (size_t row = 0; row < 2; row++)
        {
            /* code */
            int num = col * 2 + row;
            leg_sim->K(row, col) = K_coeff(num, 0) * pow(leg_sim->L0.now, 3) +
                                   K_coeff(num, 1) * pow(leg_sim->L0.now, 2) +
                                   K_coeff(num, 2) * leg_sim->L0.now +
                                   K_coeff(num, 3);
        }
    }
    // 状态更新
    leg_sim->dis.set += v_set * dt;
    leg_sim->Xd << 0, 0, leg_sim->dis.set, 0, 0, 0;
    leg_sim->X << leg_sim->angle0.now, leg_sim->angle0.dot, leg_sim->dis.now, leg_sim->dis.dot, pitch.now, pitch.dot;
    u = leg_sim->K * (leg_sim->Xd - leg_sim->X); // u = K(Xd-X);
    leg_sim->TWheel_set = u(0, 0);
    leg_sim->Tp_set = u(1, 0);
    leg_L->TWheel_set = leg_sim->TWheel_set / 2.0;
    leg_R->TWheel_set = leg_sim->TWheel_set / 2.0;

    leg_L->Tp_set = leg_sim->Tp_set / 2.0;
    leg_R->Tp_set = leg_sim->Tp_set / 2.0;
    // pid补偿
    float out_L, out_R, out_roll, out_spilt, out_turn;
    out_L = leg_L->supportF_pid.compute(leg_L->L0.set, 0, leg_L->L0.now, leg_L->L0.dot, dt);
    out_R = leg_R->supportF_pid.compute(leg_R->L0.set, 0, leg_R->L0.now, leg_R->L0.dot, dt);
    out_roll = roll_pid.compute(roll.set, 0, roll.now, roll.dot, dt);
    out_spilt = split_pid.compute(0, 0, leg_L->angle0.now - leg_R->angle0.now, leg_L->angle0.dot - leg_R->angle0.dot, dt);
    out_turn = turn_pid.compute(yaw.set, 0, yaw.now, yaw.dot, dt);
    // 左腿VMC解算
    leg_L->L0.dot = (leg_L->L0.now - leg_L->L0.last) / dt;
    leg_L->L0.last = leg_L->L0.now;
    leg_L->F_set += out_L;
    leg_L->F_set += out_roll;
    leg_L->Tp_set += out_spilt; // 这里的正负号没研究过，完全是根据仿真工程上得来的（其实这样也更快）
    Torque_L = leg_L->VMC(leg_L->F_set, leg_L->Tp_set);
    leg_L->TL_set = -Torque_L(0, 0);
    leg_L->TR_set = Torque_L(1, 0);
    // 右腿VMC解算
    leg_R->L0.dot = (leg_R->L0.now - leg_R->L0.last) / dt;
    leg_R->L0.last = leg_R->L0.now;
    leg_R->F_set += out_R;
    leg_R->F_set -= out_roll;
    leg_R->Tp_set -= out_spilt;
    Torque_R = leg_R->VMC(leg_R->F_set, leg_R->Tp_set);
    leg_R->TL_set = -Torque_R(0, 0);
    leg_R->TR_set = Torque_R(1, 0);
}

void MyRobot::run()
{
    static int last_key;

    static PID_Controller turn_pid(12, 0, 0.3, 0);
    static PID_Controller roll_pid(0.18, 0, 0.02, 0);

    pitch.now = imu->getRollPitchYaw()[1];
    pitch.dot = gyro->getValues()[2];
    roll.now = imu->getRollPitchYaw()[0];
    roll.dot = gyro->getValues()[0];
    yaw.now = imu->getRollPitchYaw()[2];
    yaw.dot = gyro->getValues()[1];
    float robot_x = gps->getValues()[0];
    pitch.now -= balance_angle; // 得到相对于平衡pitch的角度

    if (yaw.now - yaw.last > 1.5 * PI)
        yaw.now += yaw.now - yaw.last - 2 * PI;
    else if (yaw.now - yaw.last < -1.5 * PI)
        yaw.now += yaw.now - yaw.last + 2 * PI;
    else
        yaw.now += yaw.now - yaw.last;

    yaw.last = yaw.now;
    if (time == 0)
        yaw.set = yaw.now;

    // 时序更新
    time = getTime();

    int key = mkeyboard->getKey();
    while (key > 0)
    {
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
                yaw.set = yaw.now - 0.1f;
            else
                yaw.set = yaw.now - 0.2f;
            break;
        case kboard.LEFT:
            if (ABS(velocity_set) > 0.1)
                yaw.set = yaw.now + 0.1f;
            else
                yaw.set = yaw.now + 0.2f;
            break;
        case kboard.END:
            velocity_set = 0;
            break;
        case 'W':
            leg_L.L0.set += 0.0002f;
            leg_R.L0.set += 0.0002f;
            break;
        case 'S':
            leg_L.L0.set -= 0.0002f;
            leg_R.L0.set -= 0.0002f;
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
    leg_L.L0.set = Limit(leg_L.L0.set, 0.35, 0.2);
    leg_R.L0.set = Limit(leg_R.L0.set, 0.35, 0.2);

    status_update(&leg_simplified, &leg_L, &leg_R, this->pitch, this->roll, this->yaw, time_step / 1000.f, velocity_set);

    static int tick = 0;
    if (tick == 0)
    {
        /* code */
        BL_legmotor->setTorque(leg_L.TL_set);
        FL_legmotor->setTorque(leg_L.TR_set);
        BR_legmotor->setTorque(leg_R.TL_set);
        FR_legmotor->setTorque(leg_R.TR_set);
    }
    else
    {
        tick--;
    }

    L_Wheelmotor->setTorque(leg_L.TWheel_set);
    R_Wheelmotor->setTorque(leg_R.TWheel_set);

    // ofstream outfile;
    // outfile.open("data2.dat", ios::trunc);
    // outfile << time << ' ' << pitch << ' ' << disL_dot << ' ' << robot_x << ' ' << L_Torque << endl;
    // outfile.close();
}
