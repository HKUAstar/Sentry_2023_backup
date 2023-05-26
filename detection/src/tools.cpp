#include <iostream>
#include <roborts_msgs/GimbalAngle.h>
#include <roborts_msgs/ShootCmd.h>
#include <number_recognition/img_msg.h>
#include "tools.h"

Controller::Controller()
{
    ros::NodeHandle n;
    pub = n.advertise<roborts_msgs::GimbalAngle>("/cmd_gimbal_angle", 10);
    client = n.serviceClient<roborts_msgs::ShootCmd>("/cmd_shoot");

    count = 0;
}

void Controller::shoot(int mode, int number)
{
    roborts_msgs::ShootCmd srv;
    srv.request.mode = mode;
    srv.request.number = number;
    count++;
    
    if (client.call(srv))
        std::cout << "Shoot service call successful." << std::endl;
    else
        std::cout << "Shoot service call unsuccessful." << std::endl;
}

void Controller::endshoot()
{
    roborts_msgs::ShootCmd srv;
    srv.request.mode = 0;
    srv.request.number = 0;
    if (client.call(srv))
        std::cout << "End shoot service call successful." << std::endl;
    else
        std::cout << "Shoot service call unsuccessful." << std::endl;
}

void Controller::moveGimbal(double yaw_angle, double pitch_angle, double pitch_offset, bool absolute)
{
    roborts_msgs::GimbalAngle msg;
    if (absolute)
        msg.yaw_mode = 0;
    else
        msg.yaw_mode = 1;
    if (absolute)
        msg.pitch_mode = 0;
    else
        msg.pitch_mode = 1;
    msg.yaw_angle = yaw_angle;
    msg.pitch_angle = 0.0; // pitch_angle - pitch_offset;
    std::cout << "Move gimbal: " << yaw_angle << " " << pitch_angle << std::endl;
    pub.publish(msg);
}

void pixel_to_cam(cv::Mat &p_img, cv::Mat &camera_mtx, cv::Mat &camera_dist, cv::Mat &rvec, cv::Mat &tvec, int id) {
    std::vector<cv::Point3f> p_obj;
    if (id == 1 || id == 7)
    {
        p_obj.push_back(cv::Point3f(0.115, 0.0635, 0.));
        p_obj.push_back(cv::Point3f(0.115, -0.0635, 0.));
        p_obj.push_back(cv::Point3f(-0.115, -0.0635, 0.));
        p_obj.push_back(cv::Point3f(-0.115, 0.0635, 0.));
    }
    else
    {
        p_obj.push_back(cv::Point3f(0.0675, 0.0625, 0.));
        p_obj.push_back(cv::Point3f(0.0675, -0.0625, 0.));
        p_obj.push_back(cv::Point3f(-0.0675, -0.0625, 0.));
        p_obj.push_back(cv::Point3f(-0.0675, 0.0625, 0.));
    }
    cv::solvePnP(p_obj, p_img, camera_mtx, camera_dist, rvec, tvec);
}