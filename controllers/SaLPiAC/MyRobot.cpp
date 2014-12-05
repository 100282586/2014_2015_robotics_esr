/**
 * @file    MyRobot.cpp
 * @brief   Controller that "Search and Locate People in Adverse Conditions"
 * @return	void
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-12-07
 */

#include "MyRobot.h"

//////////////////////////////////////////////

// Constructor
MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 50;

    // Initial value os speeds
    _left_speed = 0;
    _right_speed = 0;

    _mode = TOP;                            // Initial move mode
    _diag_dir=NONE;                         // Initial value for diagonal

    // Initial values for angles
    DESIRED_ANGLE=45;
    initial_angle=0;

    // Initial values for flags and counters
    flag=0, flag2=0, flag3=0, flag4=0;
    pers=0;
    seconds=0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    // Get and enable the front camera
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);

    // Get size of images for forward camera
    image_width_f = _forward_camera->getWidth();
    image_height_f = _forward_camera->getHeight();

    // Get the distance sensors
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

    // Enable distance sensors
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
    {
        _distance_sensor[i]->enable(_time_step);
    }
}


//////////////////////////////////////////////

// Destructor
MyRobot::~MyRobot()
{
    _my_compass->disable();

    _forward_camera->disable();

    for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
    {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////

// Main function
void MyRobot::run()
{
    // Initial values for distance sensors
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
    {
        dist[i]=0.0;
    }

    // Initial values for green pixels counter
    for (int n=0; n<4; n++)
    {
        npix[n]=0;
    }

    // Execution loop
    while (step(_time_step) != -1)
    {
        read_angle();                       // Get the robot angle and direction

        read_distances();                   // Get the values of distance sensors

        if (pers<2)                         // If the robot haven't found 2 people yet, look for them
        {
            // Read pixels of the image divided in 4 vertical quadrants
            read_f_image(0, image_width_f/4, 0, image_height_f, 0);
            read_f_image(image_width_f/4, image_width_f/2, 0, image_height_f, 1);
            read_f_image(image_width_f/2, image_width_f-(image_width_f/4), 0, image_height_f, 2);
            read_f_image(image_width_f-(image_width_f/4), image_width_f, 0, image_height_f, 3);
        }

        if (flag2==1)                       // If the robot has a people in front
        {
            if (flag3==0)                   // If the robot positioned just now in front of the person
            {
                // Stop the robot
                _left_speed = 0;
                _right_speed = 0;

                time1=timer();              // Get actual time
                flag3++;
            }

            else if (flag3>0)               // If the robot was positioned previouly in front of the person
            {
                time2=timer();              // Get actual time

                seconds = difftime(time2,time1);        // Take the difference (in seconds) of times

                if (seconds>1.99)           // If the robot was stopped in front of the person for 2 seconds
                {
                    // Reset
                    flag3=-1;
                    seconds=0;
                }
            }

            else                            // Pivot 360
            {
                turn_around();
            }
        }

        else if ((pers==1)&&(flag2==-1))    // If the robot located a single person and already pivoted 360
        {
            turn_180();                     // Pivot 180 to leave the person behind
        }

        // If it's not pivoting 180 yet, and it's locked up in a corner, pivot 180 degrees
        else if ((flag4==0)&&((dist[0]<800)||(dist[15]<800))&&(_diag_dir!=NONE)&&((dist[3]<1000)||(dist[4]<1000))&&((dist[12]<1000)||(dist[11]<1000)))
        {
            flag4=1;                        // Set the flag to say that the robot must pivot 180
            turn_180();                     // Call the function to pivot 180
        }

        else if (flag4!=0)                  // If the robot must pivot 180
        {
            turn_180();                     // Call the function to pivot 180
        }

        // If the robot is not locked up and doesn't detect any people, move and avoid obstacles
        else
        {
            // If there is a wall in front of the robot
            if ((dist[0]<700)||(dist[15]<700))
            {
                setSpeed(0, 0);             // Stop

                // If there is no obstacles behind the robot
                if ((dist[6]>500)&&(dist[7]>500)&&(dist[8]>500)&&(dist[9]>500))
                {
                    if (_direction==RIGHT)  // If the robot is looking to the right
                    {
                        _mode=LEFT_WALL_FOLLOWER;
                    }
                    else
                    {
                        _mode=RIGHT_WALL_FOLLOWER;
                    }
                }
                else
                {
                    _mode=TURN;
                }
            }

            // If the robot detect a wall close to its left side and it's looking to the right
            else if ((dist[1]<750)&&(_direction==RIGHT))
            {
                // If there is no obstacles behind the robot
                if ((dist[6]>500)&&(dist[7]>500)&&(dist[8]>500)&&(dist[9]>500))
                {
                    // If there is any wall too close to any side of the robot
                    if ((dist[14]<850)||(dist[1]<700))
                    {
                        setSpeed(0, 0);     // Stop
                        _mode=LEFT_WALL_FOLLOWER;
                    }
                    else
                    {
                        _mode=TURN_RIGHT;
                    }
                }
                else
                {
                    _mode=TURN;
                }
            }

            // If there is a wall close to the left side of the robot, and there is space on the right
            else if ((dist[1]<750)&&(dist[14]>750))
            {
                setSpeed(0, 0);     // Stop
                _mode=TURN_RIGHT;
            }

            // If the robot detect a wall close to its right side and it's looking to the left
            else if ((dist[14]<750)&&(_direction==LEFT))
            {
                // If there is no obstacles behind the robot
                if ((dist[6]>500)&&(dist[7]>500)&&(dist[8]>500)&&(dist[9]>500))
                {
                    // If there is any wall too close to any side of the robot
                    if ((dist[1]<850)||(dist[14]<700))
                    {
                        setSpeed(0, 0);     // Stop
                        _mode=RIGHT_WALL_FOLLOWER;
                    }
                    else
                    {
                        _mode=TURN_LEFT;
                    }
                }
                else
                {
                    _mode=TURN;
                }
            }

            // If there is a wall close to the right side of the robot, and there is space on the left
            else if ((dist[14]<750)&&(dist[1]>750))
            {
                setSpeed(0, 0);     // Stop
                _mode=TURN_LEFT;
            }

            // If the robot is looking to the left and there is no obstacles on its right side or vice versa
            else if (((_direction==LEFT)&&(dist[14]>950)&&(dist[13]>950)&&(dist[12]>950)&&(dist[11]>950))||((_direction==RIGHT)&&(dist[1]>950)&&(dist[2]>950)&&(dist[3]>950)&&(dist[4]>950)))
            {
                _mode=TOP;
            }

            else
            {
                _mode=FORDWARD;
            }

            // Send actuators commands according to the mode
            switch (_mode)
            {
            case TURN_LEFT:
                _left_speed = MAX_SPEED / 1.9;
                _right_speed = MAX_SPEED;
                break;
            case TURN_RIGHT:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED / 1.9;
                break;
            case RIGHT_WALL_FOLLOWER:
                _left_speed = -MAX_SPEED / 3.0;
                _right_speed = -MAX_SPEED / 20.0;
                break;
            case LEFT_WALL_FOLLOWER:
                _left_speed = -MAX_SPEED / 20.0;
                _right_speed = -MAX_SPEED / 3.0;
                break;
            case FORDWARD:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
                break;
            case TURN:
                if (_direction==RIGHT)                      // If the robot it's looking right
                {
                    _left_speed = MAX_SPEED/5;
                    _right_speed = -MAX_SPEED/5;
                }
                else
                {
                    _left_speed = -MAX_SPEED/5;
                    _right_speed = MAX_SPEED/5;
                }
                break;
            case TOP:
                go_top();
                break;
            default:
                break;
            }
        }

        setSpeed(_left_speed, _right_speed);                // Set motor speeds
    }
}

///////////////////////////////////////////////

void MyRobot::read_angle()
{
    // Get compass value
    const double *compass_value = _my_compass->getValues();

    // Convert the compass value to degrees
    double rad = atan2(compass_value[0], compass_value[2]);
    compass_angle = rad * (180.0 / M_PI);

    if (pers==0)            // If the robot hasn't located any people yet, set the top of the map on 45 degrees
    {
        // If the robot is looking to an angle between 270 and 360, or between 0 and 45 degrees
        if (((compass_angle>=270.0)&&(compass_angle<361.0))||((compass_angle>=-1.0)&&(compass_angle<=45.1))||((compass_angle<=0)&&(compass_angle>=-90)))
        {
            _direction=LEFT;
        }
        // If the robot is looking to an angle between 45 and 222 degrees
        else if (((compass_angle>45.1)&&(compass_angle<222))||((compass_angle<=-138)&&(compass_angle>=-180)))
        {
            _direction=RIGHT;
        }

        // If the robot is looking to an angle between 350 and 360, or 0 and 10 degrees
        if (((compass_angle>-10)&&(compass_angle<10))||(compass_angle>350)||((compass_angle>260)&&(compass_angle<280))||((compass_angle>-100)&&(compass_angle<-80))||((compass_angle>80)&&(compass_angle<100))||((compass_angle>170)&&(compass_angle<190))||(compass_angle<-170))
        {
            _diag_dir=DIAG;
        }
        else
        {
            _diag_dir=NONE;
        }
    }

    else                    // If the robot has located at least 1 person, set the top of the map on 225 degrees
    {
        // If the robot is looking to an angle between 225 and 360 degrees
        if (((compass_angle>=225.0)&&(compass_angle<361.0))||((compass_angle<=0)&&(compass_angle>=-135)))
        {
            _direction=RIGHT;
        }
        // If the robot is looking to an angle between 42 and 225 degrees
        else if (((compass_angle>42.1)&&(compass_angle<225))||((compass_angle<-135)&&(compass_angle>=-180)))
        {
            _direction=LEFT;
        }

        _diag_dir=DIAG;
    }
}

//////////////////////////////////////////////

void MyRobot::read_distances()
{
    // Read the distance sensors to any object
    for (int n=0; n<NUM_DISTANCE_SENSOR; n++)
    {
        dist[n]=1000-(_distance_sensor[n]->getValue());
    }
}

//////////////////////////////////////////////

void MyRobot::go_top()
{
    double positive_angle=compass_angle;

    if (positive_angle<0)       // If the angle it's negative, set it positive
    {
        positive_angle=compass_angle+360;
    }

    if (flag>0)                 // If the robot is seeing any people, go to meet him with a simple bang-bang movement
    {
        if (positive_angle < DESIRED_ANGLE)
        {
            // Turn right
            _left_speed = MAX_SPEED/1.5;
            _right_speed = MAX_SPEED/2;
        }

        else if (positive_angle > DESIRED_ANGLE)
        {
            // Turn left
            _left_speed = MAX_SPEED/2;
            _right_speed = MAX_SPEED/1.5;
        }

        else
        {
            // Move straight forward
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED;
        }
    }

    else                        // If the robot is not seeing any people, go straight with a simple bang-bang movement
    {
        if (_direction==RIGHT)
        {
            // Turn left
            _left_speed = MAX_SPEED - 25;
            _right_speed = MAX_SPEED;
        }

        else
        {
            // Turn right
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED - 30;
        }
    }
}

//////////////////////////////////////////////

void MyRobot::read_f_image(int xo, int xf, int yo, int yf, int quad)
{
    unsigned char green = 0, red = 0, blue = 0;         // Variables to save the RGB components of each pixel

    // Get current image from forward camera
    const unsigned char *image_f = _forward_camera->getImage();

    // Reset
    npix[quad]=0;
    flag=0;

    // Get RGB components of each pixel of the quadrant
    for (xo; xo < xf; xo++)
    {
        for (yo; yo < yf; yo++)
        {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, xo, yo);
            red = _forward_camera->imageGetRed(image_f, image_width_f, xo, yo);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, xo, yo);

            // Conditions to count a pixel like green: Inverted cone (vertex below) cut at a height of 30 points of green component
            if ((green > 30) && (((green-30)/red) >= 1.2) && (((green-30)/blue) >= 1.2))
            {
                npix[quad]++;
            }
        }
    }

    // If the robot has already analized the 4 quadrants and has detected any people, take his angle
    if ((quad==3)&&((npix[0]>5)||(npix[1]>5)||(npix[2]>5)||(npix[3]>5)))
    {
        get_person_angle();
        flag=1;                         // Set the flag to know that it's seeing a person
    }

    // If the robot has de person in front of it
    if ((npix[1]>20)&&(npix[2]>20)&&(dist[0]<1000)&&(dist[15]<1000))
    {
        // If it's the first time
        if (flag2==0)
        {
            flag2=1;                    // Set the flag to know that in front of the robot there is a person
            flag=0;                     // Reset flag
        }
    }
}

//////////////////////////////////////////////

void MyRobot::get_person_angle()
{
    // Angle of each quadrant
    angle[0]=compass_angle-16.875;
    angle[1]=compass_angle-5.625;
    angle[2]=compass_angle+5.625;
    angle[3]=compass_angle+16.875;

    // If any angle is negative, make it positive
    for (int n=0;n<4;n++)
    {
        if (angle[n]<0)
        {
            angle[n]=angle[n]+360;
        }
    }

    // If there is green in the first quadrant
    if (npix[0]>5)
    {
        // If there is green in the second quadrant but not in the third neither fourth
        if ((npix[1]>5)&&(npix[2]<5)&&(npix[3]<5))
        {
            percent=npix[0]/(npix[0]+npix[1]);                          // Calculate the percent of green on the first quadrant relative to the second
            DESIRED_ANGLE=(angle[0]*percent)+(angle[1]*(1-percent));    // Calculate the angle of the person
            DESIRED_ANGLE=DESIRED_ANGLE-10;                             // Angle fix
        }
        // If there is green in the third quadrant but not in the fourth
        else if ((npix[2]>5)&&(npix[3]<5))
        {
            percent=npix[0]/(npix[0]+npix[2]);
            DESIRED_ANGLE=(angle[0]*percent)+(angle[2]*(1-percent));
            DESIRED_ANGLE=DESIRED_ANGLE-14;                             // Angle fix
        }
        // If there is green in the fourth quadrant
        else if (npix[3]>5)
        {
            percent=npix[0]/(npix[0]+npix[3]);
            DESIRED_ANGLE=(angle[0]*percent)+(angle[3]*(1-percent));
            DESIRED_ANGLE=DESIRED_ANGLE-17;                             // Angle fix
        }
        else
        {
            DESIRED_ANGLE=angle[0]-2.5;                                 // Angle fix
        }
    }

    // If there is green in the second quadrant
    else if (npix[1]>5)
    {
        // If there is green in the third quadrant but not in the fourth
        if ((npix[2]>5)&&(npix[3]<5))
        {
            percent=npix[1]/(npix[1]+npix[2]);
            DESIRED_ANGLE=(angle[1]*percent)+(angle[2]*(1-percent));
        }
        // If there is green in the fourth quadrant
        else if (npix[3]>5)
        {
            percent=npix[1]/(npix[1]+npix[3]);
            DESIRED_ANGLE=(angle[1]*percent)+(angle[3]*(1-percent));
        }
        else
        {
            DESIRED_ANGLE=angle[1];
        }
    }

    // If there is green in the third quadrant
    else if (npix[2]>5)
    {
        // If there is green in the fourth quadrant
        if (npix[3]>5)
        {
            percent=npix[2]/(npix[2]+npix[3]);
            DESIRED_ANGLE=(angle[2]*percent)+(angle[3]*(1-percent));
        }
        else
        {
            DESIRED_ANGLE=angle[2];
        }
    }

    // If there is green in the fourth quadrant
    else if (npix[3]>5)
    {
        DESIRED_ANGLE=angle[3];
    }
}

//////////////////////////////////////////////

void MyRobot::turn_around()
{
    double positive_angle=0, acu_angle=0;

    read_angle();

    if (initial_angle==0)                           // If is the first time that the robot enter in this function
    {
        initial_angle=compass_angle;                // Save tha actual angle like the initial angle

        if (initial_angle<0)                        // If the angle is negative, make it positive
        {
            initial_angle=compass_angle+360;
        }
    }

    positive_angle=compass_angle;                   // Save the actual angle

    if (positive_angle<0)                           // If the angle is negative, make it positive
    {
        positive_angle=compass_angle+360;
    }

    acu_angle=initial_angle-(positive_angle+1);

    // Turn until actual angle == 0 degrees
    if (acu_angle<0)
    {
        _left_speed = MAX_SPEED/4;
        _right_speed = -MAX_SPEED/4;
    }

    // Turn until (actual angle + 5) is above than the initial angle
    else if ((positive_angle+5)<initial_angle)
    {
        _left_speed = MAX_SPEED/4;
        _right_speed = -MAX_SPEED/4;
    }

    else
    {
        flag2=-1;                                   // Set this value (-1) means that the robot has already pivoted 360
        initial_angle=0;                            // Reset initial angle
        pers++;                                     // Add 1 to the number of located people

        if (pers==2)                                // If the robot has already located the 2 people
        {
            // Stop the robot
            _left_speed = 0;
            _right_speed = 0;

            //Reset the other flags and variables
            flag=0;
            flag3=0;
            flag4=0;
            init_angle_2=0;
        }

        cout << "People located: " << pers << endl;
    }
}

//////////////////////////////////////////////

time_t MyRobot::timer()
{
    time_t timer;                                   // Variable of type time_t to save the actual time

    time(&timer);                                   // Get the time and save it on "timer"

    return timer;                                   // Return the time
}

//////////////////////////////////////////////

void MyRobot::turn_180()
{
    double positive_angle=0;

    read_angle();

    if (initial_angle==0)                           // If is the first time that the robot enter in this function
    {
        initial_angle=compass_angle;                // Save tha actual angle like the initial angle

        if (initial_angle<0)                        // If the angle is negative, make it positive
        {
            initial_angle=compass_angle+360;
        }
    }

    positive_angle=compass_angle;                   // Save the actual angle

    if (positive_angle<0)                           // If the angle is negative, make it positive
    {
        positive_angle=compass_angle+360;
    }

    init_angle_2=initial_angle;                     // Backup of the initial angle value

    if (initial_angle<180)                          // If the initial value is less that 180 degrees
    {
        init_angle_2=initial_angle+360;
        positive_angle=positive_angle+360;
    }

    if (positive_angle>540)                         // If the actual angle is above 540 (360+180) degrees
    {
        positive_angle=positive_angle-360;
    }

    // Pivot until the robot turn 180 degrees
    if (positive_angle>(init_angle_2-180))
    {
        _left_speed = -MAX_SPEED/4;
        _right_speed = MAX_SPEED/4;
    }

    else
    {
        // Stop
        _left_speed = 0;
        _right_speed = 0;

        // Reset all flags and variables of angles
        flag=0;
        flag2=0;
        flag3=0;
        flag4=0;
        initial_angle=0;
        init_angle_2=0;
    }
}

//////////////////////////////////////////////
