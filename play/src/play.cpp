#include <ros/ros.h>
#include <roborts_msgs/GimbalAngle.h>
#include "tools.h"
using namespace std;

const double PITCH_OFFSET = 0.000;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play");

    auto controller = Controller();

    string s;

    while (ros::ok())
    {
        controller.shoot(1);
        getline(cin, s);
        ros::spinOnce();
    }

    return 0;
}