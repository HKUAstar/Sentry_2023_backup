import rospy as ros
class Referee:
    def __init__(self):
        self.robotHP = ros.Publisher('/robotHP', list, queue_size = 10)
        self.robotLV = ros.Publisher('/robotLV', list, queue_size = 10)
        self.matchBegin = ros.Publisher('/matchBegin', list, queue_size = 10)
    def unpack(self, rx_data):
        cmd_id = (rx_data[6] << 8) | rx_data[5]
        match cmd_id:
            case 1:
                self.matchBegin.publish(rx_data[8] >= 4)
            case 3:
                robotHealth = []
                for i in range(7, 35, 2):
                    robotHealth.insert((rx_data[i] << 8) + rx_data[i+1])
                self.robotHP.publish(robotHealth)
            case 513:
                robotLevel = []
                for i in range(1, 15):
                    robotLevel.insert(rx_data[(i > 7) * 9 + i])
                self.robotLV.publish(robotLevel)
