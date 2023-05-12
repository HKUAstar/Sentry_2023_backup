#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math


angularVel = 0.0
inp = " "
input_recorded = False
def recordAngularVel(msg):
    global angularVel
    angularVel = msg.angular_velocity.z

def recordInput(msg):
    global inp
    inp = msg.data

class shake_control:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.speed = 0.15
        self.turn_speed = 0.35
        self.angle = 0.0
        self.K_Vel = 1.0

        self.deltaTime = 0.05
        self.angularVel = 0.0
        self.angle = 0.0
        self.ideal_angle = 0.78

        self.maxVel = 5.0
        self.acc = 1.0
        self.vel = 1.0
        self.state = 1 # 1 for going left, -1 for going right
        self.i_constrain = 7.0
        self.d_constrain = 1.0

        self.Kp = 0.2
        self.Ki = 0.35
        self.Kd = 0.8
        self.prevError = self.state * self.ideal_angle - self.angle 
        #errorSum = prevError * Kp / Ki
        self.errorSum = -4.0

        self.control_state = 0 # 0 for not shaking, 1 for shaking, -1 for returning from shaking
        self.return_turn_speed = 0.5
        self.exit_speed = 0.3

    def shake(self):
        self.error = self.state * self.ideal_angle - self.angle 
        self.errorSum += self.error
        if self.errorSum > self.i_constrain:
            self.errorSum = self.i_constrain
        elif self.errorSum < -self.i_constrain:
            self.errorSum = -self.i_constrain
    
        self.errorDiff = (self.error - self.prevError)/self.deltaTime
        if self.errorDiff > self.d_constrain:
            self.errorDiff = self.d_constrain
        elif self.errorDiff < -self.d_constrain:
            self.errorDiff = -self.d_constrain
    
        self.prevError = self.error
        self.vel = self.Kp * self.error + self.Ki * self.errorSum + self.Kd * self.errorDiff
    
        if self.state == 1 and (self.error < 0.15 and self.error > -0.15):
            self.state = -1
            self.errorDiff /= 2
        elif self.state == -1 and (self.error < 0.15 and self.error > -0.15):
            self.state = 1
            self.errorDiff /= 2
    
        self.vel = self.K_Vel * self.vel
        if self.vel > self.maxVel:
            self.vel = self.maxVel
        elif self.vel < -self.maxVel:
            self.vel = -self.maxVel
        rospy.loginfo("ang: %.2f, angV: %.2f, vel: %.2f, state: %d, error: %.2f, P: %.2f, I: %.2f, D: %.2f",self.angle,self.angularVel,self.vel,self.state,self.error,self.Kp*self.error,self.Ki*self.errorSum,self.Kd*self.errorDiff)

    def reset_shake_param(self):
        self.vel = 1.0
        self.state = 1 # 1 for going left, -1 for going right
        self.prevError = self.state * self.ideal_angle - self.angle 
        self.errorSum = -4.0

    def changeVelocity(self,inp):
        self.inp = inp
        rospy.loginfo("%s",self.inp)
        if self.inp == 'w':
            self.x += math.cos(self.angle) * self.speed
            self.y -= math.sin(self.angle) * self.speed
        elif self.inp == 's':
            self.x -= math.cos(self.angle) * self.speed
            self.y += math.sin(self.angle) * self.speed
        elif self.inp == 'a':
            self.x += math.sin(self.angle) * self.speed
            self.y += math.cos(self.angle) * self.speed
        elif self.inp == 'd':
            self.x -= math.sin(self.angle) * self.speed
            self.y -= math.cos(self.angle) * self.speed
        elif self.inp == 'q':
            self.x = 0.0
            self.y = 0.0
        if self.control_state == 0 and self.inp == 'n':
            self.control_state = 1
        elif self.control_state == 1 and self.inp == 'p':
            self.control_state = -1
            self.reset_shake_param()

    def exit_shake(self):
        if self.angle < 0.1 and self.angle > -0.1:
            return True
        elif self.angle < 0:
            self.exit_speed = self.return_turn_speed
            return False
        elif self.angle > 0:
            self.exit_speed = -self.return_turn_speed
            return False

if __name__=="__main__":

    rospy.init_node("keyb_move_plus")

    control = shake_control()

    sub = rospy.Subscriber("keyb_input",String,recordInput,queue_size=100)
    sub_angVel = rospy.Subscriber("/wit/imu0",Imu,recordAngularVel,queue_size= 10)
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=100)

    rate = rospy.Rate(1/control.deltaTime)
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0


    while not rospy.is_shutdown():

        control.angularVel = angularVel
        control.angle += control.angularVel * control.deltaTime 

        control.changeVelocity(inp)
        inp = ' '

        msg.linear.x = control.x;
        msg.linear.y = control.y;

        if control.control_state == 1:
            control.shake()
            msg.angular.z = control.vel
        elif control.control_state == 0:
            msg.angular.z = 0
        elif control.control_state == -1:
            if control.exit_shake():
                control.control_state = 0
            msg.angular.z = control.exit_speed
            
        rospy.loginfo("x: %.2f ,y: %.2f, z: %.2f, angle: %.2f ",control.x,control.y,control.z,control.angle)
        pub.publish(msg)
        rate.sleep()




