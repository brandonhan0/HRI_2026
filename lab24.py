import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, TwistStamped

import geometry_msgs.msg
import cv2
import numpy as np


import sys
import threading

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty



moveBindings = {
    'g': (1, 0, 0, 0),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

class CameraThing(Node):
    def __init__(self):
        super().__init__('camera_node')
        node = rclpy.create_node('teleop_twist_keyboard')

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, '/example_image', 10)
        self.subscription = self.create_subscription(Image, '/oakd/rgb/image_raw', self.callback, 10)
        self.create_timer(1, self.control_loop)
        self.create_timer(1, self.keyboard_loop)
        self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)
        self.direction = 1
        self.forward_msg = TwistStamped()
        self.forward_msg.twist.linear.x = float(0.2) #1 m/s
        self.is_red = False


        # parameters
        self.stamped = node.declare_parameter('stamped', True).value
        frame_id = node.declare_parameter('frame_id', '').value
        if not self.stamped and frame_id:
            raise Exception("'frame_id' can only be set when 'stamped' is True")

        if self.stamped:
            TwistMsg = geometry_msgs.msg.TwistStamped
        else:
            TwistMsg = geometry_msgs.msg.Twist

        self.pub = node.create_publisher(TwistMsg, '/gobilda/cmd_vel', 10)

        spinner = threading.Thread(target=rclpy.spin, args=(node,))
        spinner.start()

        speed = 0.5
        turn = 1.0
        x = 0.0
        y = 0.0
        z = 0.0
        th = 0.0
        status = 0.0

        self.twist_msg = TwistMsg()

        if self.stamped:
            twist = self.twist_msg.twist
            self.twist_msg.header.stamp = node.get_clock().now().to_msg()
            self.twist_msg.header.frame_id = frame_id
        else:
            twist = self.twist_msg


    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        print("B:", img[0][0])
        print("G:", img[1][0])
        print("R:", img[2][0])

        lower_red = np.array([20, 0, 150]) # test this
        upper_red = np.array([50, 50, 200]) # test this too

        mask = cv2.inRange(img, lower_red, upper_red) # produces int > 0 if true

        self.is_red = np.any(mask>0)

        print(self.is_red)

    def control_loop(self):
        if self.is_red: # goes forward if is_red is true
             self.forward_msg.twist.linear.x = -1.0
             self.publisher_.publish(self.forward_msg)
    
    def keyboard_loop(self):
        settings = saveTerminalSettings()
        key = getKey(settings)
        if key in moveBindings.keys():
            x = moveBindings[key][0]
            y = moveBindings[key][1]
            z = moveBindings[key][2]
            th = moveBindings[key][3]
        else:
            x = 0.0
            y = 0.0
            z = 0.0
            th = 0.0


        if self.stamped:
            self.twist_msg.header.stamp = node.get_clock().now().to_msg()

        self.twist.linear.x = x
        self.twist.linear.y = y
        self.twist.linear.z = z 
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = th 
        self.pub.publish(self.twist_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraThing()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
