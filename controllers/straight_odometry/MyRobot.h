/**
 * @file    MyRobot.cpp
 * @brief   Controller that go straight with odometry.
 * @return
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-11-11
 */

//Libraries
#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;


//SETTING MAX SPEED
#define MAX_SPEED       80

// Declaration class of my controller
class MyRobot : public DifferentialWheels {
private:

    //Encoders
    double _left_encoder, _right_encoder;

    //Speeds
    double _left_speed, _right_speed;

    //Distances
    double _distance, desired_distance;

    //useful variable
    int _time_step;

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
