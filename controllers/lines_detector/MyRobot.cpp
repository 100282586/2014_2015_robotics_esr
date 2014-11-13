/**
 * @file    MyRobot.cpp
 * @brief   Controller that detect the initial and the final yellow lines with its spherical camera.
 * @return
 * @author  Eduardo Sanz Ruzafa <100282586@alumnos.uc3m.es>
 * @date    2014-11-13
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    //Initial values for speeds
    _left_speed = 0;
    _right_speed = 0;

    //Get and enable the spherical camera
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    int sum = 0;
    unsigned char green = 0, red = 0, blue = 0;

    // Get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

    while (step(_time_step) != -1) {
        sum = 0;

        // Get current image from spherical camera
        const unsigned char *image_s = _spherical_camera->getImage();

        // Count number of pixels that are yellow (or mostly yellow)
        for (int x = 0; x < image_width_s; x++) {
            for (int y = 0; y < image_height_s; y++) {
                green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

                //Tipical values for yelow colors
                if ((green > 220) && (red > 220) && (blue < 35)) {
                    sum++;
                }
            }
        }

        //If there are more than 300 yellow pixels
        if  (sum>300) {
            cout << endl << "There is a yellow line close" << endl;
        }
        else {
            cout << endl << "There is NO yelow lines close" << endl;
        }

        // Go ahead
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////
