#!/usr/bin/python3

from train import Net
import cv2
import rospy as ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import torch
import torchvision.transforms
from number_recognition.srv import img_msg, img_msgRequest, img_msgResponse

class Recognition:
    def __init__(self) -> None:
        self.device = device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = Net()
        path = "/home/astar/catkin_ws/src/number_recognition/src/Model.pt"
        self.model = torch.load(path, map_location=self.device)
        self.model.eval()
        self.model.to(device)
    
    def toTensor(self, img) -> torch.Tensor:
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="mono8")     # sensor_msgs/Image -> cv_img
        cv_img = cv2.resize(cv_img, dsize=(32, 32))       # Resize to fit the model input size
        trans = torchvision.transforms.ToTensor()
        tensor_img = trans(cv_img)*255            # cv_img -> tensor
        return tensor_img

    def inference(self, req: img_msgRequest) -> img_msgResponse:
        img = self.toTensor(req.image)
        p = self.model(img)
        num = int(p.argmax())+1
        res = img_msgResponse(num)
        ros.loginfo(str(num))
        return res

    def run(self) -> None:
        ros.init_node('server_img')
        s = ros.Service('Recognition', img_msg, self.inference)
        ros.loginfo('Service Running')
        # print(self.device)
        ros.spin()

if __name__ == '__main__':
    r = Recognition()
    r.run()