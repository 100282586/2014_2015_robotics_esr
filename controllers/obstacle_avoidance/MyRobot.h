/**
 * @file    main.cpp
 * @brief   A simple example for avoid obstacles.
 *
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-10
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       60
#define DESIRED_ANGLE   45.0
#define NUM_DISTANCE_SENSOR 16
#define DISTANCE_LIMIT      650

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        Compass * _my_compass;
        double _left_speed, _right_speed;
        DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

        enum Mode {
            STOP,
            FORWARD,
            TURN_LEFT,
            TURN_RIGHT,
            RIGHT_WALL_FOLLOWER,
            LEFT_WALL_FOLLOWER,
            GO_STRAIGHT
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
         */
        void run();

        /**
          * @brief An example for converting bearing vector from compass to angle (in degrees).
          */
        double convert_bearing_to_degrees(const double* in_vector);
};
