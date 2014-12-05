/**
 * @file    MyRobot.h
 * @brief   Controller that "Search and Locate People in Adverse Conditions"
 * @return	void
 * @author	Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-12-07
 */

// Libraries
#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>
#include <stdio.h>
#include <time.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

// Setting maximun speed and number of distance sensors
#define MAX_SPEED       65
#define NUM_DISTANCE_SENSOR 16

/**
 * @class   MyRobot
 * @brief   Class of the robot controller
 */
class MyRobot : public DifferentialWheels
{
private:
    int _time_step;

    // Compass and sensors
    Compass * _my_compass;
    DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

    // Front Camera
    Camera * _forward_camera;
    int image_width_f;
    int image_height_f;
    int npix[4];                            // Number of green pixels of each quadrant
    double angle[4];                        // Angle of each quadrant of the image
    int percent;                            // Percetage of pixels of each quadrant

    time_t time1, time2;
    double seconds;                         // Difference of times

    int flag;								// Flag that it's activated when the robot is seeing a person
    int flag2;								// Flag to know if the robot has a person in front
    int flag3;								// Flag to know if the robot positioned just now or previously in front of the person
    int flag4;                              // Flag to control the pivoting 180 when the robot it's locked up
    int pers;								// Number of people located

    // Speeds
    double _left_speed, _right_speed;

    double dist[NUM_DISTANCE_SENSOR];		// Distance of objects
    double compass_angle;					// Angle of the robot
    int DESIRED_ANGLE;						// Desired angle of the robot
    double initial_angle, init_angle_2;     // Initial angles to control 180 and 360 turns

    /**
     * @enum    Mode
     * @brief   Move modes
     */
    enum Mode
    {
        TURN_LEFT,							// Turn left mode
        TURN_RIGHT,							// Turn right mode
        RIGHT_WALL_FOLLOWER,				// Follow the right wall
        LEFT_WALL_FOLLOWER,					// Follow the left wall
        FORDWARD,							// Go straight
        TURN,								// Pivot
        TOP									// Go in a desired direction
    };
    Mode _mode;

    /**
     * @enum    Dir
     * @brief   Direction of the robot
     */
    enum Dir
    {
        LEFT,                               // The robot it's looking to the left of the map
        RIGHT                               // The robot it's looking to the right of the map
    };
    Dir _direction;

    /**
     * @enum    Diag_Dir
     * @brief   Direction of the robot if it's in diagonal
     */
    enum Diag_Dir
    {
        DIAG,								// The robot its in diagonal
        NONE								// The robot it's not in diagonal
    };
    Diag_Dir _diag_dir;

public:
    /**
     * @brief Constructor of the class
     */
    MyRobot();

    /**
     * @brief Destructor of the class
     */
    ~MyRobot();

    /**
     * @brief Function for initializing and running the template class
     * @return void
     */
    void run();

    /**
     * @brief Function to get the robot angle (in degrees) and the direction (left, right, an diagonal)
     * @return void
     */
    void read_angle();

    /**
     * @brief Function to get the distance of the sensors to any object
     * @return void
     */
    void read_distances();

    /**
     * @brief Function to get the front camera image and look for green pixels
     * @param xo Initial value to x edge
     * @param xf Final value to x edge
     * @param yo Initial value to y edge
     * @param yf Final value to x edge
     * @param quad Quadrant of the image
     * @return void
     */
    void read_f_image(int xo, int xf, int yo, int yf, int quad);

    /**
     * @brief Function to get where is the person that the robot is seeing (in degrees)
     * @return void
     */
    void get_person_angle();

    /**
     * @brief Function to pivot 360 degrees
     * @return void
     */
    void turn_around();

    /**
     * @brief Function to pivot 180 degrees
     * @return void
     */
    void turn_180();

    /**
     * @brief Function to get the time
     * @return Actual time
     */
    time_t timer();

    /**
     * @brief Function to go in a desired direction
     * @return void
     */
    void go_top();
};
