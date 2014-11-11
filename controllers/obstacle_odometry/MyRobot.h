/**
 * @file    MyRobot.h
 * @brief   Controller that void the obstacle with odometry.
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

    //Distances and angles
    double _distance, desired_distance, desired_angle;

    //Compass
    Compass* _my_compass;

    //Other useful variables
    int _time_step, _status;

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

    /**
          * @brief Function for converting bearing vector from compass to angle (in degrees).
          */
    double convert_bearing_to_degrees(const double* in_vector);
};
