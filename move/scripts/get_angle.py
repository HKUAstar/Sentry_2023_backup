#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

deltaTime = 0.1
angularVel = 0.0
angle = 0.0

def recordAngularVel(msg):
    global angularVel
    angularVel = msg.angular_velocity.z

if __name__=="__main__":

    rospy.init_node("get_angle")

    sub_angVel = rospy.Subscriber("/wit/imu0",Imu,recordAngularVel,queue_size= 10)

    rate = rospy.Rate(1/deltaTime)

    while not rospy.is_shutdown():
        angle += angularVel * deltaTime
        rospy.loginfo("angle: %.2f",angle)
        rate.sleep()
        


