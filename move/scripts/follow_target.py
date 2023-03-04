#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

x=0
distance = 0 # minus: target is at left, plus: target is at right
def callback(msg):
    global x;
    x = msg.x

if __name__=="__main__":
    rospy.init_node("follow_target_p")
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=100)
    sub = rospy.Subscriber("/target_coordinates",Vector3,callback,queue_size=100)
    rate = rospy.Rate(10)
    move = Twist()
    move.linear.x = 0.0
    move.linear.y = 0.0
    move.linear.z = 0.0
    move.angular.x = 0.0
    move.angular.y = 0.0
    move.angular.z = 0.0
    # pid
    Kp = 1.5
    Ki = 0.15
    Kd = 0.3
    prevError = 0
    errorSum = 0
    sample_time = 0.1
    center_position = 960

    while not rospy.is_shutdown():
        distance = int(x - center_position)
        if x < 0 or (distance < 25 and distance > -25):
            errorSum = 0
            prevError = 0
            move.angular.z = 0
            rospy.loginfo("angular speed at: %.2f",move.angular.z)
        else:
            rospy.loginfo("target at: %d",distance)
            error = distance/center_position# this is the distance between center of screen to position of target
            errorSum += error
            if errorSum > 300:
                errorSum = 300
            elif errorSum < -300:
                errorSum = -300
            errorDiff = (error - prevError)/sample_time
            prevError = error
            move.angular.z = Kp * error + Ki * errorSum * sample_time -Kd * errorDiff
            move.angular.z = -move.angular.z
            rospy.loginfo("angular speed at: %.2f",move.angular.z)
            if move.angular.z > 0.5:
                move.angular.z = 0.5
            if move.angular.z < -0.5:
                move.angular.z = -0.5
        pub.publish(move)
        rate.sleep()


