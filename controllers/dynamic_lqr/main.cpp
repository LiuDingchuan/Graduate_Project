/***
 * @CreatedTime   2022-04-23 17:02:54
 * @LastEditors   未定义
 * @LastEditTime  2022-04-23 19:58:16
 * @FilePath      \bishe\bishe.cpp
 */

#include "Robot.h"

int main(int argc, char **argv)
{
    ofstream outfile;
    outfile.close();
    remove("data2.dat");
    // MyRobot::get()->Wait(10);
    static int i = 0;
    while (True)
    {
        cout << "第" << ++i << "次循环" << endl;
        MyRobot::get()->MyStep();
        MyRobot::get()->run();
        outfile.open("data2.dat", ios::app);
        outfile << MyRobot::get()->getVelNow() << ' ' << MyRobot::get()->getVelSet() << endl;
        outfile.close();
    }
    return 0;
}
