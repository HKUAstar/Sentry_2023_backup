#ifndef TOOLS_H
#define TOOLS_H

#include <ros/ros.h>

class Controller
{
public:
    Controller();
    void moveGimbal(double yaw_angle, double pitch_angle, double pitch_offset);
    void shoot(int mode, int number = 1);
    void endshoot();
    bool empty();

private:
    ros::Publisher pub;
    ros::ServiceClient client;
    int count;
};

#endif