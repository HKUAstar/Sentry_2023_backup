#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

if __name__=="__main__":
    rospy.init_node("stop_move_p")
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=100)
    rate = rospy.Rate(10)
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

