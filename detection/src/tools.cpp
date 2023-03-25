#include <iostream>
#include <roborts_msgs/GimbalAngle.h>
#include <roborts_msgs/ShootCmd.h>
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

void Controller::moveGimbal(double yaw_angle, double pitch_angle, double pitch_offset)
{
    roborts_msgs::GimbalAngle msg;
    msg.yaw_mode = 1;
    msg.pitch_mode = 1;
    msg.yaw_angle = yaw_angle;
    msg.pitch_angle = pitch_angle - pitch_offset;
    pub.publish(msg);
}