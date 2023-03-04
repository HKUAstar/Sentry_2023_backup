#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import pygame


if __name__=="__main__":
    speed = 0.1
    rospy.init_node("keyb_control")
    pygame.init()
    display = pygame.display.set_mode((300,300))

    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=100)
    rate = rospy.Rate(1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    x = 0
    y = 0
    while not rospy.is_shutdown():
        keys = pygame.key.get_pressed()
        x = (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * speed
        y = (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * speed
        msg.linear.x = y
        msg.linear.y = x
        rospy.loginfo("%d, %d",y,x)
        #pub.publish(msg)
        rate.sleep()



    
