/*
 * @Description: 腿的类
 * @Version: 2.0
 * @Author: Dandelion
 * @Date: 2023-03-24 17:20:36
 * @LastEditTime: 2023-03-24 19:47:18
 * @FilePath: \webots_sim\controllers\dynamic_lqr\Leg.h
 */
#ifndef _LEG_H
#define _LEG_H

#include <Eigen>
#include "MyDefine.h"
using namespace Eigen;
class LegClass
{
private:
    float l1, l2, l3, l4, l5; // 单位是mm

public:
    // 长度
    float angle0, angle1, angle2, angle3, angle4; // 弧度制
    float L0_now;
    float L0_set;
    // 坐标(五连杆坐标系下的，原点在五连杆的中垂线上)
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
    void Zjie(float angle1, float angle4, float pitch);
    void Njie(float xc, float yc);
    Matrix<float, 2, 1> VMC(float F, float Tp);
};

#endif