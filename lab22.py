import time
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped



class GotoGoal(Node): # we dont even need this bruh idk what im doing but like itll set a timer to run the go to goal pose once
    def __init__(self):
        super().__init__('go_to_goal_pose')
        self.go = True
        if self.go:
            self.go_to_goal_pose()
            self.go = False

    def go_to_goal_pose(self):

        init_pose = PoseStamped()
        goal_pose = PoseStamped()

        nav = BasicNavigator()

        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = nav.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.orientation.z = 0.0
        init_pose.pose.orientation.w = 1.0

        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.0
        goal_pose.pose.position.y = 1.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0


        nav.setInitialPose(init_pose)
        nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

        path = nav.getPath(init_pose, goal_pose) # finds path to go to goal pose from inital pose
        #smoothed_path = nav.smoothPath(path)

        nav.goToPose(goal_pose) # actually moves to the pose
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
        if feedback.navigation_duration > 600:
            nav.cancelTask()

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            


def main(args=None):
    rclpy.init()
    poop = GotoGoal()
    poop.spin()



if __name__ == '__main__':
    main()
