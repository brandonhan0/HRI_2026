import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import time



"""
Make it turn torwards the direction of the shortest distance

"""


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


       self.create_timer(0.1, self.control_loop)
       self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)
       self.target_dist = 0.5
       self.direction = -1
       self.forward_msg = TwistStamped()
       self.forward_msg.twist.linear.x = float(0) #1 m/s


       self.turn_msg = TwistStamped()
       self.turn_msg.twist.angular.z = 0.0  #rad/s


       self.latest_scan = None

   def lidar_callback(self, msg):
       self.latest_scan = msg




   def control_loop(self):
       min_dist = float('inf')
       min_i = 0

       if self.latest_scan is None:
           self.get_logger().info(f'dying')
           return

       ranges = self.latest_scan.ranges#[540:550] # front only should be idk battery died
       for i in range(len(ranges)): # find min dist
           
           if ranges[i] < min_dist:
               if i > 120 and i < 900:
                  continue
               else:   
                  min_dist = ranges[i]
                  min_i = i
       self.get_logger().info(f'i: {min_i}')

       if min_i <120 and min_i > 0: # left
           self.turn_msg.twist.angular.z = 0.5
           self.turn_msg.twist.linear.x = 0.0 * self.direction

           self.get_logger().info(f'turning left?')

       elif min_i > 900 and min_i < 1000: # right
           self.turn_msg.twist.angular.z =  -0.5
           self.turn_msg.twist.linear.x = 0.0 * self.direction
           self.get_logger().info(f'turning right?')

       else: # forward
           self.turn_msg.twist.angular.z =  math.pi * 0.0
           self.get_logger().info(f'no turn')
           if min_dist < self.target_dist:
              self.turn_msg.twist.linear.x = -0.2 * self.direction
           elif min_dist > self.target_dist:
               self.turn_msg.twist.linear.x = 0.2 * self.direction
           else:
               self.turn_msg.twist.linear.x = 0.0

       self.get_logger().info(f'Minimum distance: {min_dist}')
      

       self.publisher_.publish(self.turn_msg) # move torwards the target
      
def main(args=None):
   rclpy.init(args=args)
   lidar_dist = LidarDist()
   rclpy.spin(lidar_dist)
