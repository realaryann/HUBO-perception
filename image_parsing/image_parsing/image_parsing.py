import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image

import torch

from cv_bridge import CvBridge
bridge = CvBridge()

model = torch.hub.load("ultralytics/yolov5", "yolov5s")

class ObjectClassifier(Node):
    def __init__(self):
        super().__init__('object_classifier')
        
        self.subscriber = self.create_subscription(Image, 'camera/rgb/image_raw', self.on_image, 10)
        self.timer = self.create_timer(0.2, self.on_timer)
        self.image = None
        self.results = None

    def on_image(self, img : Image):
        self.image = img
    
    def on_timer(self):
        if self.image:
            img = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
            self.results = model(img)
            try:
                print(self.results.xyxy)
            except Exception as e:
                print(e)
            # print(self.results.xyxy[1])
            # for obj in self.results.xyxy[0]:
            #     print (str(round(obj[2].item() - obj[0].item())) + ', ' + str(round(obj[3].item() - obj[1].item())))

def main(args=None):
    rclpy.init(args=args)
    oc = ObjectClassifier()
    rclpy.spin(oc)
    oc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()