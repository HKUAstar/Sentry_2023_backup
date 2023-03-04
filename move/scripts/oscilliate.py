#! /usr/bin/env python 
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

#pi = 3.1415926
#T = 4
#angularFreq = 2 * pi / T

deltaTime = 0.1
angularVel = 0.0
angle = 0.0
ideal_angle = 0.78

acc = 0.0
vel = 1.0
k = 1.5


def recordAngularVel(msg):
    global angularVel
    angularVel = msg.angular_velocity.z

if __name__=="__main__":
    rospy.init_node("oscilliate")
    sub_angVel = rospy.Subscriber("/wit/imu0",Imu,recordAngularVel,queue_size= 10)

    pub_vel = rospy.Publisher("/cmd_vel",Twist,queue_size=100)
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    rate = rospy.Rate(1/deltaTime)
    while not rospy.is_shutdown():
        angle += angularVel * deltaTime
        if angle  >= ideal_angle and vel > 0:
            vel = 0
        elif angle  <= ideal_angle and vel < 0:
            vel = 0
        acc = - k * angle
        vel += acc * deltaTime
        msg.angular.z = vel
        pub_vel.publish(msg)
        rospy.loginfo("ang: %.2f, angV: %.2f, acc: %.2f, vel: %.2f",angle,angularVel,acc,vel)
        rate.sleep()
