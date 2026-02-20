import time
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped



class GotoGoal(Node): # subscribes to goal pose and publishes that to ros2, we also wanna subscribe to clicked point thats the rviz point
    def __init__(self):
        super().__init__('go_to_goal_pose')

        self.publisher_goal = self.create_publisher(String, '/goal_pose', 10)
        self.publisher_iniital = self.create_publisher(String, '/initialpose', 10)

        self.create_timer(0.5, self.go_to_goal_pose)

    def go_to_goal_pose(self):


        init_pose = PoseStamped()
        goal_pose = PoseStamped()

        init_pose.header = ???
        init_pose.pose = ???

        goal_pose.header = ???
        goal_pose.pose = ???



        self.setInitialPose(init_pose)
        self.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

        path = self.getPath(init_pose, goal_pose)
        smoothed_path = self.smoothPath(path)

        self.goToPose(goal_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
        if feedback.navigation_duration > 600:
            self.cancelTask()

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            


def main(args=None):
    rclpy.init()
    nav = BasicNavigator()
    nav.spin()



if __name__ == '__main__':
    main()
