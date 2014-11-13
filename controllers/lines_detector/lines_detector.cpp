/**
 * @file    lines_detector.cpp
 * @brief   Controller that detect the initial and the final yellow lines with its spherical camera.
 * @return  control number
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-11-13
 */

#include "MyRobot.h"

//Main Program
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
