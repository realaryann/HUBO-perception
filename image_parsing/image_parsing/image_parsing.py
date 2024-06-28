import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image

from ultralytics import YOLO

from std_msgs.msg import String

from cv_bridge import CvBridge
bridge = CvBridge()

model = YOLO('yolov8n.pt')

class ObjectClassifier(Node):
    def __init__(self):
        super().__init__('object_classifier')
        
        self.subscriber = self.create_subscription(Image, 'camera/rgb/image_raw', self.on_image, 10)
        self.publisher = self.create_publisher(String, 'detected_locations', 10)
        self.timer = self.create_timer(0.2, self.on_timer)
        self.image = None
        self.results = None
        self.locactions_dict = {}

    def on_image(self, img : Image):
        self.image = img
    
    def on_timer(self):
        self.locactions_dict = {}
        if self.image:
            img = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
            self.results = model(img, verbose=False)
            for val in range(len(self.results[0].boxes.cls)):
                bounding = self.results[0].boxes.xyxy[val]
                location = (round(bounding[2].item() - bounding[0].item()), round(bounding[3].item() - bounding[1].item()))
                detected = self.results[0].names[self.results[0].boxes.cls[val].item()]
                self.locactions_dict[location] = detected
                # print(detected + " at " + str(round(location[2].item() - location[0].item())) + ', ' + str(round(location[3].item() - location[1].item())))
        msg = String()
        msg.data = str(self.locactions_dict)
        temp = msg.data
        
        self.publisher.publish(msg)
                

def main(args=None):
    rclpy.init(args=args)
    oc = ObjectClassifier()
    rclpy.spin(oc)
    oc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()