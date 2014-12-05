/**
 * @file    SaLPiAC.cpp
 * @brief   Controller that "Search and Locate People in Adverse Conditions"
 * @return	Control number without any value
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-12-07
 */

#include "MyRobot.h"

//Main program
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
