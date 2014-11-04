/**
 * @file    main.cpp
 * @brief   A simple example for avoid obstacles.
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

    _mode = GO_STRAIGHT;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    // Get and enable the distance sensors
    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[3] = getDistanceSensor("ds3");
    _distance_sensor[4] = getDistanceSensor("ds4");
    _distance_sensor[5] = getDistanceSensor("ds5");
    _distance_sensor[6] = getDistanceSensor("ds6");
    _distance_sensor[7] = getDistanceSensor("ds7");
    _distance_sensor[8] = getDistanceSensor("ds8");
    _distance_sensor[9] = getDistanceSensor("ds9");
    _distance_sensor[10] = getDistanceSensor("ds10");
    _distance_sensor[11] = getDistanceSensor("ds11");
    _distance_sensor[12] = getDistanceSensor("ds12");
    _distance_sensor[13] = getDistanceSensor("ds13");
    _distance_sensor[14] = getDistanceSensor("ds14");
    _distance_sensor[15] = getDistanceSensor("ds15");

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
//    double compass_angle, compass_angle2;
    double dist[NUM_DISTANCE_SENSOR];
    double compass_angle;

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
            dist[n]=1000-(_distance_sensor[n]->getValue());
        }

        if ((dist[12] > DISTANCE_LIMIT) && (dist[14] > DISTANCE_LIMIT) && (dist[1] > DISTANCE_LIMIT) && (dist[3] > DISTANCE_LIMIT)) {
            _mode = GO_STRAIGHT;
//            go_45();
            cout << "Tiraaa" << endl;
        }

        // Control logic of the robot
        else if ((_mode == FORWARD) || (_mode == GO_STRAIGHT)){
            // When sufficiently close to a wall in front of robot,
            // switch mode to wall following
            if ((dist[1] < DISTANCE_LIMIT-100) || (dist[14] < DISTANCE_LIMIT-100) || (dist[0] < DISTANCE_LIMIT-100) || (dist[15] < DISTANCE_LIMIT-100))  {
                if ((compass_angle < 45) || (compass_angle > 225))
                    _mode = RIGHT_WALL_FOLLOWER;
                else
                    _mode = LEFT_WALL_FOLLOWER;
            }

            else if ((compass_angle < 45) || (compass_angle > 225)) {
                if ((dist[12] > DISTANCE_LIMIT) || (dist[11] > DISTANCE_LIMIT)) {
                    _mode = TURN_RIGHT;
                    cout << "Turning right." << endl;
                }
            }
            else {
                if ((dist[3] > DISTANCE_LIMIT) || (dist[4] > DISTANCE_LIMIT)) {
                    _mode = TURN_LEFT;
                    cout << "Turning left." << endl;
                }
            }

        }
        else {
            // Wall following

            if ((dist[1] < DISTANCE_LIMIT-100) || (dist[14] < DISTANCE_LIMIT-100) || (dist[0] < DISTANCE_LIMIT-100) || (dist[15] < DISTANCE_LIMIT-100)) {
                if ((compass_angle < 45) || (compass_angle > 225))
                    _mode = RIGHT_WALL_FOLLOWER;
                else
                    _mode = LEFT_WALL_FOLLOWER;
            }

            else {
                //If the robots is turnning right
                if (_mode == TURN_RIGHT) {
                    if ((compass_angle < 45) || (compass_angle > 225)) {
                        if ((dist[12] < DISTANCE_LIMIT-50) || (dist[11] < DISTANCE_LIMIT-50)){
                            _mode = TURN_LEFT;
                            cout << "Turning left." << endl;
                        }
                        else {
                            if ((dist[12] > DISTANCE_LIMIT) || (dist[11] > DISTANCE_LIMIT)){
                                _mode = TURN_RIGHT;
                                cout << "Turning right." << endl;
                            }
                            else {
                                _mode = FORWARD;
                                cout << "Moving forward." << endl;
                            }
                        }
                    }
                    else {
                        if ((dist[3] < DISTANCE_LIMIT-50) || (dist[4] < DISTANCE_LIMIT-50)){
                            _mode = TURN_RIGHT;
                            cout << "Turning left." << endl;
                        }
                        else {
                            if ((dist[3] > DISTANCE_LIMIT) || (dist[4] > DISTANCE_LIMIT)){
                                _mode = TURN_LEFT;
                                cout << "Turning right." << endl;
                            }
                            else {
                                _mode = FORWARD;
                                cout << "Moving forward." << endl;
                            }
                        }
                    }
                }

                //If the robot isn't turnnig right
                else {
                    if ((compass_angle < 45) || (compass_angle > 225)) {
                        if ((dist[12] < DISTANCE_LIMIT-50) && (dist[11] < DISTANCE_LIMIT-50)){
                            _mode = TURN_LEFT;
                            cout << "Turning left." << endl;
                        }
                        else {
                            if ((dist[12] > DISTANCE_LIMIT) || (dist[11] > DISTANCE_LIMIT)){
                                _mode = TURN_RIGHT;
                                cout << "Turning right." << endl;
                            }
                            else {
                                _mode = FORWARD;
                                cout << "Moving forward." << endl;
                            }
                        }
                    }
                    else {
                        if ((dist[3] < DISTANCE_LIMIT-50) && (dist[4] < DISTANCE_LIMIT-50)){
                            _mode = TURN_RIGHT;
                            cout << "Turning left." << endl;
                        }
                        else {
                            if ((dist[3] > DISTANCE_LIMIT) || (dist[4] > DISTANCE_LIMIT)){
                                _mode = TURN_LEFT;
                                cout << "Turning right." << endl;
                            }
                            else {
                                _mode = FORWARD;
                                cout << "Moving forward." << endl;
                            }
                        }
                    }
                }
            }
        }

        // Send actuators commands according to the mode
        switch (_mode){
            case STOP:
                _left_speed = 0;
                _right_speed = 0;
                break;
            case FORWARD:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
                break;
            case TURN_LEFT:
                _left_speed = MAX_SPEED / 1.2;
                _right_speed = MAX_SPEED;
                break;
            case TURN_RIGHT:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED / 1.2;
                break;
            case RIGHT_WALL_FOLLOWER:
                _left_speed = -MAX_SPEED / 3.0;
                _right_speed = -MAX_SPEED / 20.0;
                break;
            case LEFT_WALL_FOLLOWER:
                _left_speed = -MAX_SPEED / 20.0;
                _right_speed = -MAX_SPEED / 3.0;
                break;
            case GO_STRAIGHT:
                // Simple bang-bang control
                if (compass_angle < (DESIRED_ANGLE - 1)) {
                    // Turn right
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED - 35;
                }
                else {
                    if (compass_angle > (DESIRED_ANGLE + 1)) {
                        // Turn left
                        _left_speed = MAX_SPEED - 35;
                        _right_speed = MAX_SPEED;
                    }
                    else {
                        // Move straight forward
                        _left_speed = MAX_SPEED;
                        _right_speed = MAX_SPEED;
                    }
                }
                break;
            default:
                break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);

    }
}

///////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////
