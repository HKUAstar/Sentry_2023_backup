#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

x = 0.0
y = 0.0
z = 0.0
speed = 0.15
turn_speed = 0.35
angle = 0.0
def changeVelocity(msg):
    global x
    global y
    global z
    inp = msg.data
    rospy.loginfo("%s",inp)
    if inp == 'w':
        x += speed
    elif inp == 's':
        x -= speed
    elif inp == 'a':
        y += speed
    elif inp == 'd':
        y -= speed
    elif inp == 'l':
        z -= turn_speed
    elif inp == 'j':
        z += turn_speed
    elif inp == 'k':
        z = 0.0
    elif inp == 'q':
        x = 0.0
        y = 0.0
        z = 0.0

if __name__=="__main__":
    rospy.init_node("keyb_move")
    sub = rospy.Subscriber("keyb_input",String,changeVelocity,queue_size=100)
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
        msg.linear.x = x;
        msg.linear.y = y;
        msg.angular.z = z;
        angle += z * 1/10
        rospy.loginfo("x: %.2f ,y: %.2f, z: %.2f, angle: %.2f ",x,y,z,angle)
        pub.publish(msg)
        rate.sleep()

