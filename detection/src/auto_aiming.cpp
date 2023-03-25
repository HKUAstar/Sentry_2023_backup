#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <msgs/Armors.h>
#include <msgs/ArmorItem.h>
#include "plain_detect.h"
#include "tools.h"
#include "auto_aiming.h"
using namespace cv;
using namespace std;

// Camera parameters: /opt/MVS/Samples/64/Python/ParametrizeCamera_LoadAndSave

/*
Modes:
0 - Velocity
1 - Acceleration
2 - ???
*/

void AutoAiming::shootTarget(Armor tar)
{
    if (SHOOT_X > tar.upper_l.x && SHOOT_X < tar.lower_r.x && 
        SHOOT_Y > tar.upper_l.y && SHOOT_Y < tar.lower_r.y)
        controller->shoot(1);
}

Armor AutoAiming::pickTarget(vector<Armor> armors)
{
    Armor tar = Armor();
    int max_area = 0;
    vector<double> dists = {};
    if (armors.size() == 0)
        return tar;
    //cout << "Armors detected: " << armors.size() << endl;
    for (size_t i = 0; i < armors.size(); i++)
    {
        if (armors[i].area() > max_area)
        {
            max_area = armors[i].area();
            tar = armors[i];
        }
        if (last_tar.center.x != 0 && last_tar.center.y != 0)
            dists.emplace_back(point_distance(last_tar.center, tar.center));
    }
    
    if (last_tar.center.x != 0 && last_tar.center.y != 0)
    {
        int last_tar_index = distance(dists.begin(), min_element(dists.begin(), dists.end()));
        //cout << "Last target: " << last_tar.center.x << " " << last_tar.center.y << endl;
        if (armors[last_tar_index].area() >= MIN_TARGET_AREA)
        {
            tar = armors[last_tar_index];
            //cout << "Current target: " << tar.center.x << " " << tar.center.y << " " << tar.area() << endl;
        }
    }

    return tar;
}

void AutoAiming::traceTarget(Armor tar)
{
    if (tar.empty())
        return;
    
    double vx = 0, vy = 0;
    double dx = 0, dy = 0;
    double ax = 0, ay = 0;

    int cx = SHOOT_X, cy = SHOOT_Y;

    pred_tar = tar.center;

    //cout << "Center: " << cx << ", Target: " << tar.upper_l.x 
    //    << "-" << tar.lower_r.x << endl;
    
    if (x_on_target && (cx - tar.lower_r.x) * last_act.x < 0 
        && (cx - tar.upper_l.x) * last_act.x < 0)
        mode = 0;
    
    if (x_on_target && (cx - tar.lower_r.x) * last_act.x > 0
        && (cx - tar.upper_l.x) * last_act.x > 0)
        mode = 1;

    if (cx >= tar.upper_l.x && cx <= tar.lower_r.x)
        x_on_target = true;
    else
        x_on_target = false;
    
    //cout << "Mode: " << ((mode == 1) ? "Acceleration" : "Velocity") << endl;

    if (mode == 1)
    {
        if (!last_tar.empty())
        {
            vx = tar.center.x - last_tar.center.x;
            vy = tar.center.y - last_tar.center.y;
        }

        pred_tar.x += vx * 30;
        pred_tar.y += vy * 30;
        
        if (abs(pred_tar.x - cx) > TARGET_ZONE)
        {
            ax = min(pow((double)abs(pred_tar.x - cx) / YAW_RANGE_2, 6), 1.0) * MAX_YAW_ACC;
            ax = max(ax, MIN_YAW_ACC);
            if (pred_tar.x > cx)
                ax = -ax;
        }

        if (abs(pred_tar.y - cy) > TARGET_ZONE)
        {
            ay = min(pow((double)abs(pred_tar.y - cy) / PITCH_RANGE, 6), 1.0) * MAX_PITCH_ACC;
            ay = max(ay, MIN_PITCH_ACC);
            if (pred_tar.y > cy)
                ay = -ay;
        }

        dx = last_act.x + ax * ACC_MULTIPLIER;
        if (abs(dx) > MAX_YAW)
            dx = MAX_YAW * ((dx > 0) ? 1 : -1);
        dy = last_act.y + ay * ACC_MULTIPLIER;
        if (abs(dy) > MAX_PITCH)
            dy = MAX_PITCH * ((dy > 0) ? 1 : -1);
    }

    if (mode == 0)
    {
        if (abs(tar.center.x - cx) > TARGET_ZONE)
        {
            dx = min(abs(tar.center.x - cx) / YAW_RANGE_1, 1.0) * MAX_YAW;
            dx = max(dx, MIN_YAW);
            if (tar.center.x > cx)
                dx = -dx;
        }

        if (abs(tar.center.y - cy) > TARGET_ZONE)
        {
            dy = min(abs(tar.center.y - cy) / PITCH_RANGE, 1.0) * MAX_PITCH;
            dy = max(dy, MIN_PITCH);
            if (tar.center.y > cy)
                dy = -dy;
        }
    }

    //cout << mode << " " << vx << " " << vy << " " << ax << " " << ay << " " << dx << " " << dy << endl;

    controller->moveGimbal(dx, -dy, PITCH_OFFSET);

    act = Point2d(dx, dy);
    //cout << "Act: " << act.x << " " << act.y << endl;
}

// publishAnnotatedImage not implemented

AutoAiming::AutoAiming()
{
    last_act = Point2d(0, 0);
    last_tar = Armor();
    x_on_target = false;
    mode = 1; // acceleration
}

void AutoAiming::aim(Mat img, char color)
{
    controller = new Controller();
    cv_bridge::CvImagePtr cv_ptr;

    //auto time0 = chrono::high_resolution_clock::now();

    vector<Armor> armors = Detector::analyze(img, color);

    //auto time1 = chrono::high_resolution_clock::now();
    //double duration = chrono::duration_cast<chrono::nanoseconds>(time1 - time0).count() / 1e9;
    //cout << "Detection: " << duration << " seconds." << endl;

    //time0 = chrono::high_resolution_clock::now();
    Armor target = pickTarget(armors);

    traceTarget(target);

    shootTarget(target);

    last_tar = target;
    last_act = act;
    //time1 = chrono::high_resolution_clock::now();
    //duration = chrono::duration_cast<chrono::nanoseconds>(time1 - time0).count() / 1e9;
    //cout << "Aiming: " << duration << " seconds." << endl;
    //cout << fixed << setprecision(9) << time1.time_since_epoch().count() / (1e9) << endl;
}

void AutoAiming::publishArmorImage(Mat img, Armor armor)
{
    Mat cropped_img = img[armor.upper_l.x:armor.lower_r.x, armor.upper_l.y:armor.lower_r.y];

    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    out_msg.image = cropped_img;
    img_pub.publish(out_msg.toImageMsg());
}

AutoAiming aim_object;
auto time_start = chrono::high_resolution_clock::now();
int aiming_cnt = 0;

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    auto start = chrono::high_resolution_clock::now();
    char color = 'r';
    
    if (aiming_cnt == 0)
        time_start = chrono::high_resolution_clock::now();

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    
    Mat img = cv_ptr->image;
    img = changeGamma(img, IMAGE_GAMMA);

    aiming_cnt++;

    aim_object.aim(img, color);
    
    auto time_end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::nanoseconds>(time_end - time_start).count() / 1e9;
    cout << "Aiming count: " << aiming_cnt << " Time elapsed: " << fixed << setprecision(9) << duration << " seconds." << endl;
    //resize(img, img, Size(img_width, img_height));
    //result_msg.img = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
}

Mat changeGamma(Mat img, double gamma)
{
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    Mat res = img.clone();
    LUT(img, lookUpTable, res);
    return res;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_aiming");
    ros::NodeHandle n;

    aim_object.img_pub = n.advertise<sensor_msgs::Image>("gamma_changed_image", 10);
    ros::Subscriber sub = n.subscribe("/hikrobot_camera/rgb", 10, callback);
    ros::spin();

    return 0;
}