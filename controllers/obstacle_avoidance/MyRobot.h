/**
 * @file    MyRobot.h
 * @brief   Controller that void obstacles.
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

//Seting speeds, desired angles, number of sensors and th distance limit
#define MAX_SPEED       60
#define DESIRED_ANGLE   45.0
#define NUM_DISTANCE_SENSOR 16
#define DISTANCE_LIMIT      650

class MyRobot : public DifferentialWheels {
private:
    int _time_step;

    //Compass and sensors
    Compass * _my_compass;
    DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

    //Speeds
    double _left_speed, _right_speed;

    //Move modes
    enum Mode {
        STOP,                       //Robot stopped
        FORWARD,                    //Go fordward
        TURN_LEFT,
        TURN_RIGHT,
        RIGHT_WALL_FOLLOWER,        //Follow the right wall
        LEFT_WALL_FOLLOWER,         //Follow the left wall
        GO_STRAIGHT                 //Try to go to the opposite side of the room
    };

    Mode _mode;

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
         * @return
         */
    void run();

    /**
          * @brief Function for converting bearing vector from compass to angle (in degrees).
          * @return Angle in degrees
          */
    double convert_bearing_to_degrees(const double* in_vector);
};
