/**
 * @file    MyRobot.cpp
 * @brief   Controller that void the obstacle with odometry.
 * @return
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-11-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    //Inital values for desired angle and distance
    desired_distance=0;
    desired_angle=0;

    //Enable encoders
    enableEncoders(_time_step);

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    //Initial values for speeds
    _left_speed = 0;
    _right_speed = 0;

    //Initial value for the distance
    _distance = 0;

    //Initial value for status of the controller
    _status=0;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    disableEncoders();

    _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;

    while (step(_time_step) != -1)
    {
        //Status
        switch (_status)
        {
        //Inilial turn to go to 90 degrees (avoiding obstacle)
        case 0:
            desired_angle=90;
            desired_distance=0;
            break;

        //Go straight with the same angle
        case 1:
            desired_distance=5.5;
            desired_angle=90;
            break;

        //Turn left to go to 45 degrees (on the right side of the obstacle)
        case 2:
            desired_angle=45;
            desired_distance=0;
            break;

        //Go straight by the right side of the obstacle
        case 3:
            desired_distance=13.5;
            desired_angle=45;
            break;

        //Default case
        default:
            desired_distance=0;
            desired_angle=0;
            break;
        }

        // Read the sensors
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        //Read the value of the encoders
        _left_encoder = getLeftEncoder();
        _right_encoder = getRightEncoder();

        //Convert the value of the left encoder to meters
        _distance = _left_encoder/60.61;

        if (_status<4)
        {
            //case 0
            if ((compass_angle<desired_angle)&&(_status==0))
            {
                _left_speed = 10;
                _right_speed = 0;
            }

            //case 2
            else if ((compass_angle>desired_angle)&&(_status==2))
            {
                _left_speed = 0;
                _right_speed = 10;
            }

            //case 1 and 3
            else if (_distance < desired_distance)
            {
                //If 1 wheel is faster than the other, we turn to go straight
                if(_left_encoder > _right_encoder)
                {
                    _left_speed = MAX_SPEED-10;
                    _right_speed = MAX_SPEED;
                }
                else
                {
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED-10;
                }
            }

            //reseting values
            else
            {
                _left_speed = 0;
                _right_speed = 0;
                setEncoders(0,0);
                _distance = 0;
                _status ++;
            }
        }

        //Setting speeds
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
