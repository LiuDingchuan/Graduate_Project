/*
 * @Description: 腿的类
 * @Version: 2.0
 * @Author: Dandelion
 * @Date: 2023-03-24 17:20:36
 * @LastEditTime: 2023-03-24 17:42:41
 * @FilePath: \dynamic_lqr\Leg.h
 */
#ifndef _LEG_H
#define _LEG_H

#include <Eigen>
using namespace Eigen;
class LegClass
{
private:
    float l1, l2, l3, l4, l5;

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

    LegClass();
    void Zjie();
    void Njie();
    Matrix<float, 2, 1> VMC();
};

#endif
