

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
    roll.set = 0, yaw.set = 0, yaw.set_dot = 0,
    roll.set = 0, yaw.set = 0, yaw.set_dot = 0,
    velocity.set = 0, velocity.now = 0;
    acc_up_max = 1.0, acc_down_max = 1.0;
    // 调参
    turn_pid.update(3.0, 0, 0.03, 3); // 针对角速度进行PD控制
    split_pid.update(100.0, 0.0, 10, 0);
    roll_pid.update(1000, 0.0, 10, 25);

    K_coeff << -230.48755905338, 296.612084269857, -162.934406813838, -27.7422261659726,
        -316.413239331338, 495.106965204476, -291.174766699827, 70.3669238069017,
        -25.7389799444334, 31.3965271835001, -27.6460735389962, -7.34997262239697,
        -94.2384846090381, 141.133985623166, -80.0319836223607, 19.1580752261303,
        -33.6318410559445, 40.9718297101160, -17.4781457291386, -7.32992836005870,
        -108.359148869453, 155.384350192654, -83.1313241956693, 17.9192945391375,
        -18.7580389710126, 21.6022273062709, -9.64342672001356, -10.0406069571186,
        -147.665355439053, 204.804728569058, -105.576426772434, 21.8884640308065,
        -383.107444865340, 549.366638576174, -293.913615344890, 63.3542734141590,
        475.626057518179, -579.429172541268, 247.178307361983, 103.660840979197,
        -13.5804334552840, 19.3130739456456, -10.5076763187910, 2.48909877341753,
        13.8114871794700, -16.9279439837071, 7.29672666195982, 3.10075600724177;
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
 * @brief: 状态更新
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
                            const float dt, float v_set)
{
    leg_L->F_set = -13.17 / 2 * 9.81;
    leg_R->F_set = -13.17 / 2 * 9.81;
    // 获取当前机器人状态信息
    leg_L->dis.now = encoder_wheelL->getValue() * 0.05;
    leg_R->dis.now = encoder_wheelR->getValue() * 0.05;
    leg_sim->dis.now = (leg_L->dis.now + leg_R->dis.now) / 2.0;

    leg_L->dis.dot = (leg_L->dis.now - leg_L->dis.last) * 1000.f / time_step;
    leg_R->dis.dot = (leg_R->dis.now - leg_R->dis.last) * 1000.f / time_step;
    leg_sim->dis.dot = (leg_sim->dis.now - leg_sim->dis.last) * 1000.f / time_step;

    leg_L->dis.last = leg_L->dis.now;
    leg_R->dis.last = leg_R->dis.now;
    leg_sim->dis.last = leg_sim->dis.now;

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
    leg_L->angle0.dot = (leg_L->angle0.now - leg_L->angle0.last) / dt;

    leg_R->angle1 = 2.0 / 3.0 * PI - encoder_FR->getValue();
    leg_R->angle4 = 1.0 / 3.0 * PI + encoder_BR->getValue();
    leg_R->Zjie(leg_R->angle1, leg_R->angle4, pitch.now);
    leg_R->angle0.dot = (leg_R->angle0.now - leg_R->angle0.last) / dt;
    // 计算K矩阵数据，根据L0.now拟合得到
    static Matrix<float, 2, 1> u;
    static Matrix<float, 2, 1> Torque_L, Torque_R;
    leg_sim->angle1 = (leg_L->angle1 + leg_R->angle1) / 2;
    leg_sim->angle4 = (leg_L->angle4 + leg_R->angle4) / 2;
    leg_sim->Zjie(leg_sim->angle1, leg_sim->angle4, pitch.now);
    leg_sim->angle0.dot = (leg_sim->angle0.now - leg_sim->angle0.last) / dt;
    // 限幅
    v_set = limitVelocity(v_set, leg_sim->L0.now);
    // pitch_dot = Deadzone(pitch_dot, 0.01);
    // leg_sim->dis.dot = Deadzone(leg_sim->dis.dot, 0.01);
    // leg_sim->angle0_dot = Deadzone(leg_sim->angle0_dot, 0.1);
    // 根据当前杆长计算K矩阵的各项值
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

    leg_L->Tp_set = -leg_sim->Tp_set / 2.0;
    leg_R->Tp_set = -leg_sim->Tp_set / 2.0;
    // pid补偿
    float out_L, out_R, out_roll, out_spilt, out_turn;
    out_roll = roll_pid.compute(roll.set, 0, roll.now, roll.dot, dt);
    out_spilt = split_pid.compute(0, 0, leg_L->angle0.now - leg_R->angle0.now, leg_L->angle0.dot - leg_R->angle0.dot, dt);
    out_turn = turn_pid.compute(yaw.set_dot, 0, yaw.dot, yaw.ddot, dt);
    // 左腿VMC解算
    leg_L->L0.dot = (leg_L->L0.now - leg_L->L0.last) / dt;
    leg_L->L0.last = leg_L->L0.now;
    out_L = leg_L->supportF_pid.compute(leg_L->L0.set, 0, leg_L->L0.now, leg_L->L0.dot, dt);
    leg_L->F_set -= out_L;
    leg_L->F_set -= out_roll;
    leg_L->Tp_set -= out_spilt; // 这里的正负号没研究过，完全是根据仿真工程上得来的（其实这样也更快）
    Torque_L = leg_L->VMC(leg_L->F_set, leg_L->Tp_set);
    leg_L->TL_set = -Torque_L(0, 0);
    leg_L->TR_set = Torque_L(1, 0);
    leg_L->TWheel_set -= out_turn;
    // 右腿VMC解算
    leg_R->L0.dot = (leg_R->L0.now - leg_R->L0.last) / dt;
    leg_R->L0.last = leg_R->L0.now;
    out_R = leg_R->supportF_pid.compute(leg_R->L0.set, 0, leg_R->L0.now, leg_R->L0.dot, dt);
    leg_R->F_set -= out_R;
    leg_R->F_set += out_roll;
    leg_R->Tp_set += out_spilt;
    Torque_R = leg_R->VMC(leg_R->F_set, leg_R->Tp_set);
    leg_R->TL_set = -Torque_R(0, 0);
    leg_R->TR_set = Torque_R(1, 0);
    leg_R->TWheel_set += out_turn;
}

void MyRobot::run()
{
    time = getTime();

    static int last_key;

    velocity.now = gps->getSpeed();
    pitch.now = imu->getRollPitchYaw()[1];
    pitch.dot = gyro->getValues()[2];
    roll.now = imu->getRollPitchYaw()[0];
    roll.dot = gyro->getValues()[0];
    yaw_get = imu->getRollPitchYaw()[2];
    float yaw_dot_last = yaw.dot;
    yaw.dot = gyro->getValues()[1];
    yaw.ddot = (yaw.dot - yaw_dot_last) / (time_step * 0.001);
    float robot_x = gps->getValues()[0];
    pitch.now -= balance_angle; // 得到相对于平衡pitch的角度

    if (yaw_get - yaw.last > 1.5 * PI)
        yaw.now += yaw_get - yaw.last - 2 * PI;
    else if (yaw.now - yaw.last < -1.5 * PI)
        yaw.now += yaw_get - yaw.last + 2 * PI;
    else
        yaw.now += yaw_get - yaw.last;

    yaw.last = yaw.now;
    if (time == 0)
        yaw.set = yaw.now;

    // 时序更新

    int key = mkeyboard->getKey();
    while (key > 0)
    {
        Keyboard kboard;
        switch (key)
        {
        case kboard.UP:
            velocity.set += 0.02f;
            break;
        case kboard.DOWN:
            velocity.set -= 0.02f;
            break;
        case kboard.RIGHT:
            if (ABS(velocity.set) > 0.1)
                yaw.set_dot -= 0.01f;
            else
                yaw.set_dot -= 0.02f;
            break;
        case kboard.LEFT:
            if (ABS(velocity.set) > 0.1)
                yaw.set_dot += 0.01f;
            else
                yaw.set_dot += 0.02f;
            break;
        case kboard.END:
            velocity.set = 0;
            yaw.set_dot = 0;
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
            roll.set -= 0.002f;
            break;
        case 'D':
            roll.set += 0.002f;
            break;
        case 'O':
            sampling_flag = 1;
            sampling_time = time;
            break;
        case ' ':
            if (last_key != key)
                stop_flag = True;
            break;
        }
        last_key = key;
        key = mkeyboard->getKey();
    }
    leg_L.L0.set = Limit(leg_L.L0.set, 0.35, 0.2);
    leg_R.L0.set = Limit(leg_R.L0.set, 0.35, 0.2);
    /*测试用的，追踪一个持续4s的速度期望*/
    // if (sampling_flag == 1)
    // {
    //     if (time - sampling_time < 1)
    //     {
    //         velocity.set = 0;
    //     }
    //     else if (time - sampling_time < 5)
    //     {
    //         // velocity.set += acc_up_max * time_step * 0.001;
    //         velocity.set = 1.0;
    //     }
    //     // else if (time < 6)
    //     // {
    //     //     velocity.set = 3;
    //     // }
    //     // else if (time < 9)
    //     // {
    //     //     velocity.set -= acc_down_max * time_step * 0.001;
    //     // }
    //     else
    //     {
    //         velocity.set = 0;
    //         sampling_flag = 0;
    //     }
    // }

    velocity.set = Limit(velocity.set, 2.5, -2.5);
    yaw.set_dot = Limit(yaw.set_dot, 2.0, -2.0);
    yaw.set += yaw.set_dot * time_step * 0.001;
    status_update(&leg_simplified, &leg_L, &leg_R, this->pitch, this->roll, this->yaw, time_step * 0.001, velocity.set);

    /*输出力矩*/
    BL_legmotor->setTorque(leg_L.TL_set);
    FL_legmotor->setTorque(leg_L.TR_set);
    BR_legmotor->setTorque(leg_R.TL_set);
    FR_legmotor->setTorque(leg_R.TR_set);

    L_Wheelmotor->setTorque(leg_L.TWheel_set);
    R_Wheelmotor->setTorque(leg_R.TWheel_set);

    // ofstream outfile;
    // outfile.open("data2.dat", ios::trunc);
    // outfile << time << ' ' << pitch << ' ' << disL_dot << ' ' << robot_x << ' ' << L_Torque << endl;
    // outfile.close();
}
/**
 * @brief: 限制速度
 * @author: Dandelion
 * @Date: 2023-04-18 19:45:57
 * @param {float} L0
 * @return {*}
 */
float MyRobot::limitVelocity(float speed_set, float L0)
{
    float speed_max = -30 * L0 + 10.7;
    return (speed_set > speed_max) ? speed_max : speed_set;
}
