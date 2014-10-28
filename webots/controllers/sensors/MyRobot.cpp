/**
 * @file    main.cpp
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 50;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    // Get and enable the distance sensors
    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[3] = getDistanceSensor("ds3");
    _distance_sensor[4] = getDistanceSensor("ds12");
    _distance_sensor[5] = getDistanceSensor("ds13");
    _distance_sensor[6] = getDistanceSensor("ds14");
    _distance_sensor[7] = getDistanceSensor("ds15");

    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->enable(_time_step);
    }
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _my_compass->disable();

    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////


void MyRobot::run()
{
    double compass_angle;
    double dist[NUM_DISTANCE_SENSOR];

    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        dist[i]=0.0;
    }
	
    while (step(_time_step) != -1) {
        // Read the sensors
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Read the distances
        for (int n=0; n<NUM_DISTANCE_SENSOR; n++) {
            dist[n]=_distance_sensor[n]->getValue();
        }

        // Print sensor values to console
        for (int n=0; n<NUM_DISTANCE_SENSOR; n++) {
            cout << "Distance Sensor " << n << "= " << dist[n] << endl;
        }

        // Simple bang-bang control
        if (compass_angle < (DESIRED_ANGLE - 1)) {
            // Turn right
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED - 5;
        }
        else {
            if (compass_angle > (DESIRED_ANGLE + 1)) {
                // Turn left
                _left_speed = MAX_SPEED - 5;
                _right_speed = MAX_SPEED;
            }
            else {
                // Move straight forward
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
            }
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);

    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////
