/*
 * @Description: Leg内部函数定义
 * @Version: 2.0
 * @Author: Dandelion
 * @Date: 2023-03-24 17:19:53
 * @LastEditTime: 2023-04-01 00:04:29
 * @FilePath: \webots_sim\controllers\dynamic_lqr\Leg.cpp
 */
#include "Leg.h"

LegClass::LegClass()
{
    l1 = 0.180;
    l2 = 0.200;
    l3 = 0.200;
    l4 = 0.180;
    l5 = 0.120;
    F_set = 15.245 / 2 * 9.8;
    dis = dis_desire = dis_dot = dis_last = 0;

    xc = 0, yc = 0.28817;
    angle0 = 0;
    angle1 = PI / 3 * 2;
    angle4 = PI / 3;
    Zjie(angle1, angle4, 0);
    L0_set = L0_now; // 初值
    L0_last = L0_now;
    K << -46.9417, -8.1097, -21.5150, -17.8734, 19.2631, 1.2306,
        22.3368, 4.1576, 12.1831, 9.7243, 136.0725, 4.2612;
    X << 0, 0, 0, 0, 0, 0;
    supportF_pid.update(1.0, 0.0, 0.01, 0);
}
/**
 * @brief: 
 * @author: Dandelion
 * @Date: 2023-03-31 23:50:02
 * @param {float} xc
 * @param {float} yc
 * @return {*}
 */
void LegClass::Njie(const float xc, const float yc)
{
    this->xc = xc;
    this->yc = yc;

    float m, n, b, x1, y1;
    float A, B, C;

    A = 2 * l1 * yc;
    B = 2 * l1 * (xc + l5 / 2);
    C = l2 * l2 - l1 * l1 - xc * xc - yc * yc - l5 * l5 / 4;
    angle1 = 2 * atan((A + sqrt(A * A + B * B - C * C)) / (B - C));
    if (angle1 < 0)
        angle1 += 2 * PI;

    // nije_5(&angle1, (void *)0, x, y, l1, l6, l3, l4, l5); //利用L1,L6计算c1;
    m = l1 * cos(angle1);
    n = l1 * sin(angle1);
    b = 0;
    // x1 = l2 / l6 * ((x - m) * cos(b) - (y - n) * sin(b)) + m;
    // y1 = l2 / l6 * ((x - m) * sin(b) + (y - n) * cos(b)) + n; //得到闭链五杆端点的坐标
    x1 = ((xc - m) * cos(b) - (yc - n) * sin(b)) + m;
    y1 = ((xc - m) * sin(b) + (yc - n) * cos(b)) + n; // 得到闭链五杆端点的坐标

    A = 2 * y1 * l4;
    B = 2 * l4 * (x1 - l5 / 2);
    // c = l3 * l3 + 2 * l5 * x1 - l4 * l4 - l5 * l5 - x1 * x1 - y1 * y1;
    C = l3 * l3 + 2 * l5 * x1 - l4 * l4 - l5 * l5 / 4 - x1 * x1 - y1 * y1;
    angle4 = 2 * atan((A - sqrt(A * A + B * B - C * C)) / (B - C));
    // nije_5((void *)0, &angle2, x1, y1, l1, l2, l3, l4, l5);        //计算c4 ,
}
/**
 * @brief: 运动学正解
 * @author: Dandelion
 * @Date: 2023-03-24 19:50:04
 * @param {float} angle1
 * @param {float} angle4
 * @param {float} pitch %rad
 * @return {*}
 */
void LegClass::Zjie(const float angle1, const float angle4, const float pitch)
{
    this->angle1 = angle1;
    this->angle4 = angle4;

    xb = l1 * cos(angle1) - l5 / 2;
    yb = l1 * sin(angle1);
    xd = l5 / 2 + l4 * cos(angle4);
    yd = l4 * sin(angle4);
    float lbd = sqrt(pow((xd - xb), 2) + pow((yd - yb), 2));
    float A0 = 2 * l2 * (xd - xb);
    float B0 = 2 * l2 * (yd - yb);
    float C0 = pow(l2, 2) + pow(lbd, 2) - pow(l3, 2);
    float D0 = pow(l3, 2) + pow(lbd, 2) - pow(l2, 2);
    angle2 = 2 * atan((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))) / (A0 + C0));
    angle3 = M_PI - 2 * atan((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(D0, 2))) / (A0 + D0));
    xc = xb + l2 * cos(angle2);
    yc = yb + l2 * sin(angle2);

    L0_now = sqrt(pow(xc, 2) + pow(yc, 2));
    // 乘以pitch的旋转矩阵
    Matrix<float, 2, 2> matrix_R;
    Matrix<float, 2, 1> cor_XY;
    Matrix<float, 2, 1> cor_XY_then;
    matrix_R << cos(pitch), -sin(pitch),
        sin(pitch), cos(pitch);
    cor_XY << xc, yc;
    cor_XY_then = matrix_R * cor_XY;
    angle0 = atan(cor_XY_then(0, 0) / cor_XY_then(1, 0));
    if(angle0 < 0.01f & angle0 > -0.01f)//限幅
        angle0 = 0.0f;
}
/**
 * @brief: VMC（虚拟力算法）
 * @author: Dandelion
 * @Date: 2023-03-27 15:27:06
 * @param {float} F 沿杆方向的受力
 * @param {float} Tp 杆所受的力矩
 * @return {*}
 */
Matrix<float, 2, 1> LegClass::VMC(float F, float Tp)
{
    this->F_set = F;
    this->Tp_set = Tp;

    Matrix<float, 2, 2> Trans;
    Matrix<float, 2, 1> VirtualF;
    Matrix<float, 2, 1> ActualF;
    Trans << l1 * sin(angle0 + angle3) * sin(angle1 - angle2) / sin(angle2 - angle3),
        -l1 * cos(angle0 + angle3) * sin(angle1 - angle2) / (L0_now * sin(angle2 - angle3)),
        l4 * sin(angle0 + angle2) * sin(angle3 - angle4) / sin(angle2 - angle3),
        -l4 * cos(angle0 + angle2) * sin(angle3 - angle4) / (L0_now * sin(angle2 - angle3));
    VirtualF << this->F_set, this->Tp_set;
    ActualF = Trans * VirtualF;
    return ActualF;
}
