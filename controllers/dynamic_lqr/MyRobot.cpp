

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
    roll.set = 0, yaw.set = 0, velocity.set = 0, velocity.now = 0;
    acc_up_max = 1.0, acc_down_max = 1.0;
    // 调参
    turn_pid.update(1.0, 0.0, 0.01, 0.1); // 针对角速度进行PD控制
    split_pid.update(5.0, 0.0, 0.5, 0);
    roll_pid.update(50, 0.0, 0.5, 5);

    K_coeff << -429.480926399657, 571.526174116835, -314.019443652643, -76.7785349323096,
        -488.342988794160, 866.554903419461, -604.589464686117, 187.374037158340,
        -39.1298616447598, 54.1668612217607, -54.6200131352883, -17.1139770542784,
        -115.547990878676, 191.388298044994, -128.196609773182, 41.2434052112891,
        -63.2278520669541, 82.3347397662138, -38.6593779466379, -15.5512990394007,
        -135.216572481170, 216.484966320896, -136.272802114251, 37.7080095081746,
        -18.4556994189321, 23.6791102688333, -11.7061092639569, -24.4451326540678,
        -254.410522452798, 368.774314958874, -207.734577177161, 51.2893902825330,
        -213.796173188225, 342.292786349747, -215.466218898256, 59.6215980376592,
        399.888048199063, -520.730616452063, 244.503374478194, 98.3550510772689,
        -12.5522846786203, 18.0661909101152, -10.3282537945708, 2.79215478157118,
        17.7525913251454, -22.7623099599658, 10.5441659120560, 2.38704789005868;
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
                            const float dt, float v_set)
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

    leg_L->Tp_set = leg_sim->Tp_set / 2.0;
    leg_R->Tp_set = leg_sim->Tp_set / 2.0;
    // pid补偿
    float out_L, out_R, out_roll, out_spilt, out_turn;
    out_roll = roll_pid.compute(roll.set, 0, roll.now, roll.dot, dt);
    out_spilt = split_pid.compute(0, 0, leg_L->angle0.now - leg_R->angle0.now, leg_L->angle0.dot - leg_R->angle0.dot, dt);
    out_turn = turn_pid.compute(yaw.set_dot, 0, yaw.dot, yaw.ddot, dt);
    // 左腿VMC解算
    leg_L->L0.dot = (leg_L->L0.now - leg_L->L0.last) / dt;
    leg_L->L0.last = leg_L->L0.now;
    out_L = leg_L->supportF_pid.compute(leg_L->L0.set, 0, leg_L->L0.now, leg_L->L0.dot, dt);
    leg_L->F_set += out_L;
    leg_L->F_set += out_roll;
    leg_L->Tp_set += out_spilt; // 这里的正负号没研究过，完全是根据仿真工程上得来的（其实这样也更快）
    Torque_L = leg_L->VMC(leg_L->F_set, leg_L->Tp_set);
    leg_L->TL_set = -Torque_L(0, 0);
    leg_L->TR_set = Torque_L(1, 0);
    leg_L->TWheel_set += out_turn;
    // 右腿VMC解算
    leg_R->L0.dot = (leg_R->L0.now - leg_R->L0.last) / dt;
    leg_R->L0.last = leg_R->L0.now;
    out_R = leg_R->supportF_pid.compute(leg_R->L0.set, 0, leg_R->L0.now, leg_R->L0.dot, dt);
    leg_R->F_set += out_R;
    leg_R->F_set -= out_roll;
    leg_R->Tp_set -= out_spilt;
    Torque_R = leg_R->VMC(leg_R->F_set, leg_R->Tp_set);
    leg_R->TL_set = -Torque_R(0, 0);
    leg_R->TR_set = Torque_R(1, 0);
    leg_R->TWheel_set -= out_turn;
}

void MyRobot::run()
{
    static int last_key;

    velocity.now = gps->getSpeed();
    pitch.now = imu->getRollPitchYaw()[1];
    pitch.dot = gyro->getValues()[2];
    roll.now = imu->getRollPitchYaw()[0];
    roll.dot = gyro->getValues()[0];
    yaw.now = imu->getRollPitchYaw()[2];
    float yaw_dot_last = yaw.dot;
    yaw.dot = gyro->getValues()[1];
    yaw.ddot = (yaw.dot - yaw_dot_last) / (time_step * 0.001);
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
            roll.set -= 0.02f;
            break;
        case 'D':
            roll.set += 0.02f;
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
    leg_L.L0.set = Limit(leg_L.L0.set, 0.32, 0.2);
    leg_R.L0.set = Limit(leg_R.L0.set, 0.32, 0.2);

    if (sampling_flag == 1)
    {
        if (time - sampling_time < 1)
        {
            velocity.set = 0;
        }
        else if (time - sampling_time < 4)
        {
            // velocity.set += acc_up_max * time_step * 0.001;
            velocity.set = 2;
        }
        // else if (time < 6)
        // {
        //     velocity.set = 3;
        // }
        // else if (time < 9)
        // {
        //     velocity.set -= acc_down_max * time_step * 0.001;
        // }
        else
        {
            velocity.set = 0;
            sampling_flag = 0;
        }
    }

    velocity.set = Limit(velocity.set, 3.5, 0);
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
