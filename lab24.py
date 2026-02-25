import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraThing(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, '/example_image', 10)
        self.subscription = self.create_subscription(Image, '/oakd/rgb/image_raw', self.callback, 10)

    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        crop = img[50:350,50:350]
        crop_img = self.bridge.cv2_to_imgmsg(crop, encoding="bgr8")
        self.img_pub.publish(crop_img)

def main(args=None):
    rclpy.init(args=args)
    node = CameraThing()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
