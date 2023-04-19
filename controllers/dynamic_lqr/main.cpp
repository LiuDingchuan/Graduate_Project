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
        if (wheel_legged.getTime() > 0.5)
        {
            outfile.open("data2.dat", ios::app);
            outfile << wheel_legged.getTime() << " "
                    << wheel_legged.leg_L.TWheel_set << " "
                    << wheel_legged.leg_R.TWheel_set << " "
                    << wheel_legged.getVelNow() << " "
                    << wheel_legged.getVelSet() << " "
                    << wheel_legged.leg_simplified.angle0.now << " "
                    << wheel_legged.leg_simplified.angle0.dot << " "
                    << wheel_legged.pitch.now << " "
                    << wheel_legged.pitch.dot << " "
                    << endl;
            outfile.close();
        }
    }
    return 0;
}
