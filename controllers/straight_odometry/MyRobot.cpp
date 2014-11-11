/**
 * @file    MyRobot.cpp
 * @brief   Controller that go straight with odometry.
 * @return
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-11-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    //Inital values for desired distance
    desired_distance=17;

    //Enable encoders
    enableEncoders(_time_step);

    //Initial values for speeds
    _left_speed = 0;
    _right_speed = 0;

    //Initial value for the distance
    _distance = 0;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    while (step(_time_step) != -1)
    {
        //Read the value of the encoders
        _left_encoder = getLeftEncoder();
        _right_encoder = getRightEncoder();
        //Convert the value of the left encoder to meters
        _distance = _left_encoder/60.61;

        if (_distance < desired_distance)
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

        //If we are over the final line, we stop
        else
        {
            _left_speed = 0;
            _right_speed = 0;
        }
        setSpeed(_left_speed, _right_speed);
    }
}
