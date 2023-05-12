#! /usr/bin/env python 
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math

K_Vel = 1.5

deltaTime = 0.05
angularVel = 0.0
angle = 0.0
ideal_angle = 0.78

maxVel = 5.0
acc = 1.0
vel = 1.0
state = 1 # 1 for going left, -1 for going right
i_constrain = 7.0
d_constrain = 1.0

Kp = 0.2
Ki = 0.35
Kd = 0.8
errorSum = - 4.0
prevError = state * ideal_angle - angle 
errorOffset = 0.2
realError = 0

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
        error = state * ideal_angle - angle 

        #if state == 1 and error > prevError + errorOffset:
        #    error = prevError + errorOffset
        #elif state == -1 and error < prevError - errorOffset:
        #    error = prevError - errorOffset

        errorSum += error
        if errorSum > i_constrain:
            errorSum = i_constrain
        elif errorSum < -i_constrain:
            errorSum = -i_constrain

        errorDiff = (error - prevError)/deltaTime
        if errorDiff > d_constrain:
            errorDiff = d_constrain
        elif errorDiff < -d_constrain:
            errorDiff = -d_constrain

        prevError = error
        vel = Kp * error + Ki * errorSum + Kd * errorDiff

        if state == 1 and (error < 0.15 and error > -0.15):
            state = -1
            errorDiff /= 2
        elif state == -1 and (error < 0.15 and error > -0.15):
            state = 1
            errorDiff /= 2

        vel = K_Vel * vel
        if vel > maxVel:
            vel = maxVel
        elif vel < -maxVel:
            vel = -maxVel

        msg.angular.z = vel
        pub_vel.publish(msg)
        rospy.loginfo("ang: %.2f, angV: %.2f, vel: %.2f, state: %d, error: %.2f, P: %.2f, I: %.2f, D: %.2f",angle,angularVel,vel,state,error,Kp*error,Ki*errorSum,Kd*errorDiff)
        rate.sleep()
