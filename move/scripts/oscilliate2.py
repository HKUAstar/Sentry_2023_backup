#! /usr/bin/env python 
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

deltaTime = 0.1
angularVel = 0.0
angle = 0.0

acc = 0.0
vel = 1.0
k = 1.3

ideal_angle = 0.78

Kp = 1.5
prevError = ideal_angle

def recordAngularVel(msg):
    global angularVel
    angularVel = msg.angular_velocity.z

if __name__=="__main__":
    rospy.init_node("oscilliate")
    sub_angVel = rospy.Subscriber("/wit/imu",Imu,recordAngularVel,queue_size= 10)

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
        if vel > 0:
            error = angle - ideal_angle
        else:
            error = angle + ideal_angle
        errorDiff = (error - prevError)/deltaTime
        prevError = error

        angle += angularVel * deltaTime
        acc = - k * angle
        vel += acc * deltaTime
        msg.angular.z = vel
        if angle  >= 0.45 and vel > 0:
            vel = 0
        elif angle  <= -0.45 and vel < 0:
            vel = 0
        pub_vel.publish(msg)
        rospy.loginfo("ang: %.2f, angV: %.2f, acc: %.2f, vel: %.2f",angle,angularVel,acc,vel)
        rate.sleep()
