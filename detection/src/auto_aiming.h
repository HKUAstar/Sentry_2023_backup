#ifndef AUTO_AIMING_H
#define AUTO_AIMING_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

// ### Hyperparameters ###
const int IMG_W = 1440;
const int IMG_H = 1080;
const double MAX_PITCH_IDLE_RANGE = 0.4;
const double PITCH_OFFSET = 0.000; // for raising the head
const double MAX_YAW = 0.100;
const double MAX_PITCH = 0.025;
const double MIN_YAW = 0.005;
const double MIN_PITCH = 0.005;
const double YAW_RANGE_1 = 700; // velocity mode
const double YAW_RANGE_2 = 250; // acceleration mode
const double PITCH_RANGE = 200;
const double MAX_YAW_ACC = 0.110;
const double MIN_YAW_ACC = 0.040;
const double MAX_PITCH_ACC = 0.035;
const double MIN_PITCH_ACC = 0.010;
const double ACC_MULTIPLIER = 0.025;
const int TARGET_ZONE = 8; // radius in pixels
const int MIN_TARGET_AREA = 5000;
//const int TARGET_AREA_EPS = 100;
//const int MODE_EPS = 3;
const double IMAGE_GAMMA = 3;
const double MAX_TARGET_FRAME_DISTANCE = 20.00;
const bool USE_ROS = false;
// ### Hyperparameters ###

const int SHOOT_X = IMG_W / 2;
const int SHOOT_Y = IMG_H / 2;

class AutoAiming
{
public:
    AutoAiming();
    void aim(Mat &img, char color);
    void shootTarget(Armor tar);
    void traceTarget(Armor tar);
    Armor pickTarget(vector<Armor> &armors);
    void matchArmors(Mat &img, vector<Armor> &armors);
    void publishArmorImage(Mat &img, Armor armor);
    void publishStatus();
    ros::Publisher armor_pub, img_pub;
    unsigned int identifyArmor(Mat &img, Armor new_armor);
    void publishArmor();
    ros::ServiceClient recognition_client;
    bool needPublish();
    int tar_id;
    // void publishAnnotatedImage(Mat img, vector<Armor> armors, vector<int> tar, 
    //     vector<int> pred_tar, Point2d act);

private:
    vector<Armor> last_armors;
    Controller* controller;
    Armor last_tar;
    Point2i pred_tar;
    Point2d last_act, act;
    vector<Armor> new_armors;
    bool x_on_target;
    int mode, missed_tar;
    double idle_angle;
};

Mat changeGamma(Mat &img, double gamma);

#endif