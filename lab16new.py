import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import random
"""  

Likelyhoods:

    P( next | current ):
             L     R    F   given
      000 | 0.33  0.33 0.33
O(t)  100 | 1.00  0.00 0.00
      010 | 0.00  0.00 1.00
      001 | 0.00  1.00 0.00
      101 | 0.50  0.50 0.00
      110 | 0.50  0.00 0.50
      011 | 0.00  0.50 0.50
      111 | 0.33  0.33 0.33
"""

outcomes = ["Left","Right","Forward"]

P = [[0.33, 0.33, 0.33],
    [1.00, 0.00, 0.00],
    [0.00, 0.00, 1.00],
    [0.00, 0.00, 1.00],
    [0.00, 1.00, 0.00],
    [0.50, 0.00, 0.50],
    [0.00, 0.50, 0.50],
    [0.33, 0.33, 0.33]]

class LidarDist(Node):
    def __init__(self):
        super().__init__('lidar_dist')
        self.subscription = self.create_subscription(  #Creating subscriber (Recieving Data)
            LaserScan, #Telling the Lidar to turn on
            '/scan',
            self.lidar_callback, #Runs "callback" when data is received
            10
        )
        self.subscription  #prevent unused variable warning


        self.create_timer(1, self.control_loop)
        self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)
        self.target_dist = 0.5
        self.direction = -1
        self.turn_msg = TwistStamped()
        self.turn_msg.twist.angular.z = 0.0  #rad/s
        self.turn_msg.twist.linear.x = 0.0
        self.latest_scan = None
        self.human = ""

    def lidar_callback(self, msg): # bc it will error if timer and lidar callback r the same bleh
        self.latest_scan = msg
       
    def control_loop(self):
        left_mean = 0
        right_mean = 0
        front_mean = 0

        
        if self.latest_scan is None:
            self.get_logger().info(f'dying')
            return

        ranges = self.latest_scan.ranges # front only should be idk battery died
        for i in range(len(ranges)): # find min dist           
            if i > 120 and i < 900: # backwards we dont care ab this
                continue
            if i <120 and i > 0: # left
                self.turn_msg.twist.angular.z = 0.5
                self.turn_msg.twist.linear.x = 0.0 * self.direction
                left_mean = (left_mean + float(ranges[i])) / 2 # calc mean for left
            elif i > 900 and i < 1000: # right
                self.turn_msg.twist.angular.z =  -0.5
                self.turn_msg.twist.linear.x = 0.0 * self.direction
                left_mean = (left_mean + float(ranges[i])) / 2 # calc mean for right
            elif i > 1000: # forward (i > 1000)
                front_mean = (front_mean + float(ranges[i])) / 2 # calc front mean
                self.turn_msg.twist.angular.z =  math.pi * 0.0
                self.turn_msg.twist.linear.x = 0.0
        cur_o = [0,0,0]
        if left_mean > 0.4 and left_mean < 0.7:
            cur_o[0] = 1
        if right_mean > 0.4 and right_mean < 0.7:
            cur_o[1] = 1
        if front_mean > 0.4 and front_mean < 0.7:
            cur_o[2] = 1

        match(cur_o):
            case ([0,0,0]):
                thing = 0
            case ([1,0,0]):
                thing = 1
            case ([0,1,0]):
                thing = 2
            case ([0,0,1]):
                thing = 3
            case ([1,0,1]):
                thing = 4
            case ([1,1,0]):
                thing = 5
            case ([0,1,1]):
                thing =6
            case ([1,1,1]):
                thing = 7
        transition_prob = random.choices(outcomes, weights=P[thing], k=1)
        print(f"next transition_prob for {P[thing]}: {transition_prob[0]}")

       #self.publisher_.publish(self.turn_msg) # turn torwards the target
      
def main(args=None):
   rclpy.init(args=args)
   lidar_dist = LidarDist()
   rclpy.spin(lidar_dist)
