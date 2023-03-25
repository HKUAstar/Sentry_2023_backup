#ifndef AUTO_AIMING_H
#define AUTO_AIMING_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

// ### Hyperparameters ###
const int IMG_W = 1440;
const int IMG_H = 1080;
const double PITCH_OFFSET = 0.025; // for raising the head
const double MAX_YAW = 0.070;
const double MAX_PITCH = 0.03;
const double MIN_YAW = 0.005;
const double MIN_PITCH = 0.005;
const double YAW_RANGE_1 = 400; // velocity mode
const double YAW_RANGE_2 = 150; // acceleration mode
const double PITCH_RANGE = 200;
const double MAX_YAW_ACC = 0.110;
const double MIN_YAW_ACC = 0.040;
const double MAX_PITCH_ACC = 0.035;
const double MIN_PITCH_ACC = 0.010;
const double ACC_MULTIPLIER = 0.025;
const int TARGET_ZONE = 8; // radius in pixels
const int MIN_TARGET_AREA = 2400;
//const int TARGET_AREA_EPS = 100;
//const int MODE_EPS = 3;
const double IMAGE_GAMMA = 2.50;
const double MIN_TARGET_CHANGE_DISTANCE = 20.00;
// ### Hyperparameters ###

const int SHOOT_X = IMG_W / 2;
const int SHOOT_Y = IMG_H / 2;

class AutoAiming
{
public:
    AutoAiming();
    void aim(Mat img, char color);
    void shootTarget(Armor tar);
    void traceTarget(Armor tar);
    Armor pickTarget(vector<Armor> armors);
    void publishArmorImage(Mat img, Armor armor);
    ros::Publisher img_pub;
    // void publishAnnotatedImage(Mat img, vector<Armor> armors, vector<int> tar, 
    //     vector<int> pred_tar, Point2d act);

private:
    Controller* controller;
    Armor last_tar;
    Point2i pred_tar;
    Point2d last_act, act;
    bool x_on_target;
    int mode;
};

Mat changeGamma(Mat img, double gamma);

#endif