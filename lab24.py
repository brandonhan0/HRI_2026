import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraThing(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, '/example_image', 10)
        self.subscription = self.create_subscription(Image, '/oakd/rgb/image_raw', self.callback, 10)
        self.create_timer(1, self.control_loop)
        self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)
        self.direction = 1
        self.forward_msg = TwistStamped()
        self.forward_msg.twist.linear.x = float(0.2) #1 m/s
        self.is_red = False


    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        crop = img[50:350,50:350]  # this is a array

        lower_red = np.array([150, 0, 0]) # test this
        upper_red = np.array([255, 50, 50]) # test this too

        mask = cv2.inRange(crop, lower_red, upper_red)

        self.is_red = np.any(mask > 0)

    def control_loop(self):
        if self.is_red: # goes forward if is_red is true
             self.forward_msg.twist.linear.x = 0.2 * self.direction
             self.publisher_.publish(self.forward_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraThing()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
