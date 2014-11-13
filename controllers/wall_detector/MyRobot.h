/**
 * @file    MyRobot.h
 * @brief   Controller that detect walls with its front camera.
 * @return
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-11-13
 */

//Libraries
#include <iostream>
#include <webots/DifferentialWheels.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace std;
using namespace webots;

//SETTING MAX SPEED
#define MAX_SPEED           50

// Declaration class of my controller
class MyRobot : public DifferentialWheels {
private:
    //Useful variable
    int _time_step;

    //Front Camera
    Camera * _forward_camera;

    //Speeds
    double _left_speed, _right_speed;

public:
    /**
         * @brief Empty constructor of the class.
         */
    MyRobot();

    /**
         * @brief Destructor of the class.
         */
    ~MyRobot();

    /**
         * @brief User defined function for initializing and running the template class.
         */
    void run();
};
