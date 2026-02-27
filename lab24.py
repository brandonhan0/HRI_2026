import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, TwistStamped
import cv2
import numpy as np
import time
import sys

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

class CameraThing(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.settings = saveTerminalSettings()

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, '/example_image', 10)
        self.subscription = self.create_subscription(Image, '/oakd/rgb/image_raw', self.callback, 10)
        self.create_timer(0.1, self.getKey)

        self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)
        self.direction = 1
        self.forward_msg = TwistStamped()
        self.forward_msg.twist.linear.x = float(0.2) #1 m/s
        self.is_red = False
        self.key = ""



    def callback(self, msg): # reads red
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        print("B:", img[0][0])
        print("G:", img[1][0])
        print("R:", img[2][0])

        lower_red = np.array([20, 0, 150]) # test this
        upper_red = np.array([50, 50, 200]) # test this too

        mask = cv2.inRange(img, lower_red, upper_red) # produces int > 0 if true

        self.is_red = np.any(mask>0)

        print(self.is_red)

        if self.is_red: # goes forward if is_red is true
            self.forward_msg.twist.linear.x = 1.0
            self.publisher_.publish(self.forward_msg)
        elif self.key == 'g':
            self.forward_msg.twist.linear.x = -1.0
            self.publisher_.publish(self.forward_msg)

    def getKey(self): # read keyboard output
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            if msvcrt.kbhit() > 0:
                self.key = msvcrt.getwch()
            else:
                self.key = ""
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            self.key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraThing()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
