#! /usr/bin/env python

import rospy as ros
import message_filters
from msgs.msg import Attack_cmd
from msgs.msg import Armors


class Decision():
    def __init__(self):
        ros.init_node("decision")

        # self.Priority_Score_System = Priority_Score_System()
        self.attack_pub = ros.Publisher('/decision/attack', Attack_cmd, queue_size=10)
        # self.planning_pub = ros.Publisher('/decision/planning', Vector3, queue_size=10)
        self.is_attacking = False
        self.cur_target = 0

        '''
        sub_detection = message_filters.Subscriber('/armors', Armors) 
        # Armors: a list of all Armor plates detected. Each armor plate contains: 3 points and the recognized number 

        sub_enemy_hp = message_filters.Subscriber('/judging',Vector3)  
        # needs a reading and decoding functions for a referee system

        sub_sync = message_filters.ApproximateTimeSynchronizer([sub_detection, sub_enemy_hp], 1, 0.1)  # check parameters
        sub_sync.registerCallback(self.gimble_control)
        '''

        sub_detection = ros.Subscriber("/armors", Armors, self.gimble_control, queue_size = 10)

        # sub_localization = message_filters.Subscriber('/localization', Vector3)
        # sub_localization.registerCallback(self.chassis_control)

        ros.spin()


    # def gimble_control(self, detection, enemy_hp):
    def gimble_control(self, detection):
        '''
        detection
           variable   |   type    |  description
            count     |   int     |  number of enemies detected
            items     |   list    |  [[area, id], [area, id], ...]   id: 1-hero, 3/4-walk, others-sentry
        enemy_hp
           variable   |   type    |  description
             hps      |   list?   |  [hp1, hp2]
        ''' 
        
        code = 0  # see Notion
        if self.is_attacking == False:
            if detection.count:
                self.is_attacking = 1
                # self.cur_target = cal_priority(detection.items, enemy_hp.hps)
                self.cur_target = detection.items[0].id
                code = 201
            else:
                code = 200
        else:
            flag = 0
            for item in detection.items:
                if item.id == self.cur_target:
                    flag=1
            if flag == 0:
                self.is_attacking = 0
                self.cur_target = 0
                code = 203
            else:
                code = 202
        msg = Attack_cmd()
        msg.code = code
        msg.target_id = self.cur_target
        ros.loginfo(msg)
        self.attack_pub.publish(msg)


    # def chassis_control(self, locolization):

    class Priority_Score_System:
        def __init__(self):
            self.RType = [3, 2, 0] #Index: 0 -> Sentry, 1 -> Hero, 2 -> Standard
            
            self.RDistance = [3, 1, -2] #Index: 0 -> close, 1 -> mid, 2 -> far
            #Exact parameters of Distances is based on the area of detected square, which still needs testing for exact metrics

            self.RHealth = [2, 0 ,-1] #Index: 0 -> low, 1 -> middle， 2 -> high

            self.RExp = [0, 1, 2] #Index: 0 -> EXP between [25,75), 1 -> [75, 100), 2 -> [100, INF)


        def getTypeScore(self,ind):
            return self.RType[ind]
        def getTypeIndex(robotNumber):
            if robotNumber == 1: return 1
            elif robotNumber == 7: return 0
            else: return 2


        def getDistanceScore(self, ind):
            return self.Sigmoid(7, )
        def getDistanceIndex(size):
            if size >= 93000:
                return 0
            elif size >=35000:
                return 1
            elif size >= 15000:
                return 2

        def getHealthScore(self, ind):
            return self.RHealth[ind]

        def getHealthIndex(Health):
            if Health <= 100: return 0
            elif Health <= 300: return 1
            else: return 2

        def getExpScore(self,ind):
            return self.RExp[ind]
        def getExpIndex(robotNumber, Level):
            exp = 25
            if robotNumber == 1:
                exp = [75, 100, 150][Level - 1]
            elif robotNumber == 7:
                exp = 75
            else: 
                exp = [25, 50, 75][Level - 1]


            if exp < 75: return 0
            elif exp < 100: return 1
            else: return 2

        #def getPriority():

        #Tool-Functions
        def calcArea(x1, y1, x2, y2, x3, y3):
            return 2 * (x1 * y2 - x1 * y3 + x2 * y3 - x2 * y1 + x3 * y1 - x2 * y2)
        #might not be required
        def Sigmoid(A, b, x):
            return A / (1 + math.exp(x)) + b

if __name__ == "__main__":
    Decision()
    
    
