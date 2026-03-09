
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import random
from sensor_msgs.msg import Image
from ultralytics import YOLO
import cv2 as cv




"""
Things we can see:

person
bicycle
car
motorbike
aeroplane
bus
train
truck
boat
traffic light
fire hydrant
stop sign
parking meter
bench
bird
cat
dog
horse
sheep
cow
elephant
bear
zebra
giraffe
backpack
umbrella
handbag
tie
suitcase
frisbee
skis
snowboard
sports ball
kite
baseball bat
baseball glove
skateboard
surfboard
tennis racket
bottle
wine glass
cup
fork
knife
spoon
bowl
banana
apple
sandwich
orange
broccoli
carrot
hot dog
pizza
donut
cake
chair
sofa
pottedplant
bed
diningtable
toilet
tvmonitor
laptop
mouse
remote
keyboard
cell phone
microwave
oven
toaster
sink
refrigerator
book
clock
vase
scissors
teddy bear
hair drier
toothbrush

"""



"""
Make it turn torwards the direction of the shortest distance

ros2 launch gobilda_robot gobilda_ekf_odom_viz.launch.py

we use a venv for yolo, to active i believe you do:

source yolovenv/bin/activate

this will put us in the venv, than we install ultralytics (pip install ultralytics)

"""


class LidarDist(Node):
    def __init__(self):
        super().__init__('lidar_dist')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.subscription  #prevent unused variable warning

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, '/oakd/rgb/image_raw', self.image_callback, 10)

        self.create_timer(0.5, self.control_loop)
        self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)
        self.target_dist = 0.33
        self.direction = -1
        self.forward_msg = TwistStamped()
        self.forward_msg.twist.linear.x = float(0) #1 m/s
        self.turn_msg = TwistStamped()
        self.turn_msg.twist.angular.z = 0.0  #rad/s

        self.turn_left_after_reverse = False
        self.turn_right_after_reverse = False

        self.goal_item = ['banana']

        self.model = YOLO("yolov8n.pt")
        self.turn_loop = 0



        self.latest_scan = None
        self.object_in_frame = False
        self.img = None



    def lidar_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def control_loop(self):
        min_dist = float('inf')
        min_i = 0
        outcomes = ["Left","Right"]
        P = [0.5, 0.5]

        if self.object_in_frame:
            self.get_logger().info(f'item has been found im not moving anymore')
            return

        # ================================ Lidar dying ==============================

        if self.latest_scan is None:
            self.get_logger().info(f'lidar dying')
            return

        # =================================== Camera dying ============================

        if self.img is None:
            self.get_logger().info(f'camera dying')
            return

        # =============== live feed pls work and give me a frame ===================

        # ret, frame = self.cap.read()
        # if not ret:
        #     print("no video dawg")

        # cv.imshow('Live Feed', frame)
        # cv.imshow('Live Feed', self.img)

        # ======================= do we see the object we want? ========================

        results = self.model(self.img)
        for result in results:
            names = [result.names[cls.item()] for cls in result.boxes.cls.int()]  # class name of each box

        for r in results:
            for c in r.boxes.cls:
                class_name = self.model.names[int(c)]
                print(f"Detected class: {class_name}")
                if class_name in self.goal_item:
                    self.object_in_frame = True
                    self.get_logger().info(f'{self.goal_item} HAS BEEN FOUND!!!!!')
                    return
        # =================================== Calculates minimum range ============================


        ranges = self.latest_scan.ranges   #[540:550] front only should be idk battery died
        for i in range(len(ranges)): # find min dist
            
            if ranges[i] < min_dist:
                if i > 120 and i < 963:
                   continue
                else:   
                   min_dist = ranges[i]
                   min_i = i
        self.get_logger().info(f'i: {min_i}')

        # ====================== actually moves robot =============================================

        if (min_i <120 and min_i > 0) or self.turn_right_after_reverse: # thimg is left, turn right
            if (self.turn_loop > 0):
                self.turn_loop -= 1
            else:
                self.turn_right_after_reverse = False

            self.turn_msg.twist.angular.z = -0.85 #-1.0
            self.turn_msg.twist.linear.x = 0.0 * self.direction
            #self.get_logger().info(f'wall is left?')

        elif (min_i > 900 and min_i < 1000) or self.turn_left_after_reverse: # thing is right, turn left
            if (self.turn_loop > 0):
                self.turn_loop -= 1
            else:
                self.turn_right_after_reverse = False
            self.turn_msg.twist.angular.z =  0.85 #1.0
            self.turn_msg.twist.linear.x = 0.0 * self.direction
            #self.get_logger().info(f'wall is right?')

        else: # thing is forward, go backwards, else go forwards
            transition_prob = random.choices(outcomes, weights=P, k=1)
            #print(f"transition_prob: {transition_prob[0]}")


            self.turn_msg.twist.angular.z =  math.pi * 0.0
            
            #self.get_logger().info(f'no turn')
            if min_dist < self.target_dist: # if were close to wall back up
               
                if transition_prob == "Left":
                    self.turn_left_after_reverse = True
                    self.turn_loop = 5

                elif transition_prob == "Right":
                    self.turn_right_after_reverse = True
                    self.turn_loop = 5
                self.turn_msg.twist.linear.x = -1.0 * self.direction #-0.2
            elif min_dist > self.target_dist: # if were far from anything go forward
                self.turn_msg.twist.linear.x = 0.2 * self.direction #0.2
            else:
                self.turn_msg.twist.linear.x = 0.0

        #self.get_logger().info(f'Minimum distance: {min_dist}')
      

        #self.publisher_.publish(self.turn_msg) # move torwards the target
      
def main(args=None):
    rclpy.init(args=args)
    lidar_dist = LidarDist()
    rclpy.spin(lidar_dist)
