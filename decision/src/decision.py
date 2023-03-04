import rospy
import matplotlib.pyplot as plt
import numpy as np 

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3

class Dictator:
    def __init__(self):
        self.loc_sub = message_filters.Subscriber('/astar/localization', Vector3)
        self.ene_sub = message_filters.Subscriber('/astar/enemy_analysis', Vector3) # todo: custom message type
        
        ts = message_filters.TimeSynchronizer([self.loc_sub, self.ene_sub], queue_size=10)
        ts.registerCallback(self.callback) # may cause potential sync issue, check later

        rospy.spin()

    def callback(self):
        

if __name__ == '__main__':
    dictator = Dictator()
