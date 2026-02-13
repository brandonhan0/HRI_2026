import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import random
import numpy as np

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

outcomes = ["Left","Right","Front"]

P = [[0.33, 0.33, 0.33],
    [1.00, 0.00, 0.00],
    [0.00, 0.00, 1.00],
    [0.00, 0.00, 1.00],
    [0.00, 1.00, 0.00],
    [0.50, 0.00, 0.50],
    [0.00, 0.50, 0.50],
    [0.33, 0.33, 0.33]]

transition_prob = random.choices(outcomes, weights=P[0], k=1)
print(f"transition_prob: {transition_prob[0]}")

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
        self.counter = 0
        self.observations = []

    def lidar_callback(self, msg): # bc it will error if timer and lidar callback r the same bleh
        self.latest_scan = msg
       

    def control_loop(self):
        
        def viterbi(obs, states, start_p, trans_p, emit_p):
            V = [{}] # list of dicts, one dict per step
            path = {} # most likely path
            for y in states: # set most likley path initally
                V[0][y] = start_p[y] * emit_p[y][obs[0]]
                path[y] = [y] # start probabilities, [y ends up being best end probability ]
            for t in range(1, len(obs)): # updates most probable path
                V.append({})
                newpath = {}
                for y in states: # backwards recusion, for each current state choose the most likely previous outcome
                    (prob, state) = max( # change most likely path based off backwards recursion calculating probability based off ending point
                                        # picks max probability and returns the max probability, and best prev state
                        [(V[t-1][y0] * trans_p[y0][y] * emit_p[y][obs[t]], y0) for y0 in states]
                    )
                    V[t][y] = prob # setting probability of most likley path, v[t][y] is the most likely path
                    newpath[y] = path[state] + [y]  # takes best prev state and adds it to path

                path = newpath

            (prob, state) = max([(V[-1][y], y) for y in states]) # final termination, picks the best "path" that has the highest prob
            return (prob, path[state]) # return the prob and the path


        left_mean = 0
        right_mean = 0
        front_mean = 0

        self.counter += 1

        
        if self.latest_scan is None:
            self.get_logger().info(f'dying')
            return

        ranges = self.latest_scan.ranges # front only should be idk battery died
        for i in range(len(ranges)): # find min dist   
            if math.isinf(ranges[i]):
                continue
            if i > 120 and i < 900: # backwards we dont care ab this
                continue
            elif i <120 and i > 0: # left
                self.turn_msg.twist.angular.z = 0.5
                self.turn_msg.twist.linear.x = 0.0 * self.direction
                left_mean = (left_mean + float(ranges[i])) / 2 # calc mean for left
            elif i > 900 and i < 1000: # right
                self.turn_msg.twist.angular.z =  -0.5
                self.turn_msg.twist.linear.x = 0.0 * self.direction
                right_mean = (right_mean + float(ranges[i])) / 2 # calc mean for right
            elif i > 1000: # forward (i > 1000)
                front_mean = (front_mean + float(ranges[i])) / 2 # calc front mean
                self.turn_msg.twist.angular.z =  math.pi * 0.0
                self.turn_msg.twist.linear.x = 0.0
        cur_o = [0,0,0]
        cur_os = "000"
        
        if left_mean > 0.4 and left_mean < 0.7:
            cur_o[0] = 1
        if right_mean > 0.4 and right_mean < 0.7:
            cur_o[1] = 1
        if front_mean > 0.4 and front_mean < 0.7:
            cur_o[2] = 1
        print(f"left_mean: {left_mean}")
        print(f"right_mean: {right_mean}")
        print(f"front_mean: {front_mean}")

        match(cur_o):
            case ([0,0,0]):
                cur_os = "000"
                thing = 0
            case ([1,0,0]):
                cur_os = "100"
                thing = 1
            case ([0,1,0]):
                cur_os = "010"
                thing = 2
            case ([0,0,1]):
                cur_os = "001"
                thing = 3
            case ([1,0,1]):
                cur_os = "101"
                thing = 4
            case ([1,1,0]):
                cur_os = "110"
                thing = 5
            case ([0,1,1]):
                cur_os = "011"
                thing =6
            case ([1,1,1]):
                cur_os = "111"
                thing = 7
        print(cur_os)



        transition_prob = random.choices(outcomes, weights=P[thing], k=1)
        #print(f"next transition_prob for {P[thing]}: {transition_prob[0]}")
        
        states = outcomes
        start_probability = {'Left': 0.33, 'Right': 0.33, 'Front': 0.33}

        transition_probability = {'Left': {'Left': 0.5, 'Right': 0.0001,'Front': 0.4999},
                                 'Right': {'Left': 0.0001, 'Right': 0.5,'Front': 0.4999},
                                 'Front': {'Left': 0.25, 'Right': 0.25,'Front': 0.5}}

        emission_probability = {'Left': {'000': 0.33, '100': 1,'010': 0, '001': 0, '101': 0.5, '110': 0.5, '011': 0, '111': 0.33},
                                'Right': {'000': 0.33, '100': 0,'010': 0, '001': 1, '101': 0.5, '110': 0, '011': 0.5, '111': 0.33},
                                'Front': {'000': 0.33, '100': 0,'010': 1, '001': 0,'101': 0, '110': 0.5, '011': 0.5, '111': 0.33}
                                }      

        if self.counter < 20:
            self.observations.append(cur_os)
        else:
            print(viterbi(self.observations, states, start_probability, transition_probability, emission_probability))
            self.counter = 0
            self.observations = []
            self.observations.append(cur_os)

        #self.publisher_.publish(self.turn_msg) # turn torwards the target
      
def main(args=None):
   rclpy.init(args=args)
   lidar_dist = LidarDist()
   rclpy.spin(lidar_dist)
