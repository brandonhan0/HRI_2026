import rospy
from sensor_msgs.msg import LaserScan


class LaserScanSplit():
    """
    Class for splitting LaserScan into three parts.
    """

    def __init__(self):

        self.update_rate = 50
        self.freq = 1./self.update_rate

        # Initialize variables
        self.scan_data = []

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # Publishers
        self.pub1 = rospy.Publisher('/scan1', LaserScan, queue_size=10)
        self.pub2 = rospy.Publisher('/scan2', LaserScan, queue_size=10)
        self.pub3 = rospy.Publisher('/scan3', LaserScan, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(self.freq), self.laserscan_split_update)

    def lidar_callback(self, msg):
        """
        Callback function for the Scan topic
        """
        self.scan_data = msg

    def laserscan_split_update(self, event):
        """
        Function to update the split scan topics
        """

        scan1 = LaserScan()
        scan2 = LaserScan()
        scan3 = LaserScan()

        scan1.header = self.scan_data.header
        scan2.header = self.scan_data.header
        scan3.header = self.scan_data.header

        scan1.angle_min = self.scan_data.angle_min
        scan2.angle_min = self.scan_data.angle_min
        scan3.angle_min = self.scan_data.angle_min

        scan1.angle_max = self.scan_data.angle_max
        scan2.angle_max = self.scan_data.angle_max
        scan3.angle_max = self.scan_data.angle_max

        scan1.angle_increment = self.scan_data.angle_increment
        scan2.angle_increment = self.scan_data.angle_increment
        scan3.angle_increment = self.scan_data.angle_increment

        scan1.time_increment = self.scan_data.time_increment
        scan2.time_increment = self.scan_data.time_increment
        scan3.time_increment = self.scan_data.time_increment

        scan1.scan_time = self.scan_data.scan_time
        scan2.scan_time = self.scan_data.scan_time
        scan3.scan_time = self.scan_data.scan_time

        scan1.range_min = self.scan_data.range_min
        scan2.range_min = self.scan_data.range_min
        scan3.range_min = self.scan_data.range_min

        scan1.range_max = self.scan_data.range_max
        scan2.range_max = self.scan_data.range_max
        scan3.range_max = self.scan_data.range_max

        # LiDAR Range
        n = len(self.scan_data.ranges)

        scan1.ranges = [float('inf')] * n
        scan2.ranges = [float('inf')] * n
        scan3.ranges = [float('inf')] * n

        # Splitting Block [three equal parts]
        scan1.ranges[0 : n//3] = self.scan_data.ranges[0 : n//3]
        scan2.ranges[n//3 : 2*n//3] = self.scan_data.ranges[n//3 : 2*n//3]
        scan3.ranges[2*n//3 : n] = self.scan_data.ranges[2*n//3 : n]

        # Publish the LaserScan
        self.pub1.publish(scan1)
        self.pub2.publish(scan2)
        self.pub3.publish(scan3)

    def kill_node(self):
        """
        Function to kill the ROS node
        """
        rospy.signal_shutdown("Done")

if __name__ == '__main__':

    rospy.init_node('laserscan_split_node')
    LaserScanSplit()
    rospy.spin()
