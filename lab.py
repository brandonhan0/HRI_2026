import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped






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
       self.turn_msg.twist.angular.z = math.pi * 0.5  #rad/s


       self.latest_scan = None






   def lidar_callback(self, msg):
       self.latest_scan = msg




   def control_loop(self):
       min_dist = float('inf')


       if self.latest_scan is None:
           self.get_logger().info(f'dying')
           return


       ranges = self.latest_scan.ranges[540:550] # front only should be idk battery died




       for i in ranges: # find min dist
           if i < min_dist:
               min_dist = i


       if min_dist < self.target_dist:
           self.forward_msg.twist.linear.x = -1.0 * self.direction
       elif min_dist > self.target_dist:
           self.forward_msg.twist.linear.x = 1.0 * self.direction
       else:
           self.forward_msg.twist.linear.x = 0.0


       self.get_logger().info(f'Minimum distance: {min_dist}')




       self.publisher_.publish(self.forward_msg)






      
def main(args=None):
   rclpy.init(args=args)
   lidar_dist = LidarDist()
   rclpy.spin(lidar_dist)
   lidar_dist.destroy_node()
   rclpy.shutdown()
