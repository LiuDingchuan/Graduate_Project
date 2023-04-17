/***
 * @CreatedTime   2022-04-23 17:02:54
 * @LastEditors   未定义
 * @LastEditTime  2022-04-23 19:58:16
 * @FilePath      \bishe\bishe.cpp
 */

#include "Robot.h"

int main(int argc, char **argv)
{
    MyRobot wheel_legged;
    ofstream outfile;
    outfile.close();
    remove("data2.dat");
    // MyRobot::get()->Wait(10);
    static int i = 0;
    while (True)
    {
        wheel_legged.MyStep();
        wheel_legged.run();
        outfile.open("data2.dat", ios::app);
        outfile << wheel_legged.leg_L.TWheel_set << " " << wheel_legged.leg_R.TWheel_set << endl;
        outfile.close();
    }
    return 0;
}
