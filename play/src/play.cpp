#include <ros/ros.h>
#include <roborts_msgs/GimbalAngle.h>
#include "tools.h"
using namespace std;

const double PITCH_OFFSET = 0.013;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play");

    auto controller = Controller();

    string s;

    while (ros::ok())
    {
        controller.moveGimbal(0.02, 0.00, 0.025);
        ros::spinOnce();
    }

    return 0;
}