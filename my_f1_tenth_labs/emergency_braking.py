#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.

        #need message type, topic name, queue_size
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriber_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscriber_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

    def odom_callback(self, odom_msg):
        # TODO: update current speed

        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        
        #We will now implement iTTC:
        #need to fix up r value to get first index in array...
        #need to update speed to have angular element...three angular elements?
        r = LaserScan.ranges
        car_speed = max(self.speed, 0)
        ITTC = r/car_speed

        # TODO: publish command to brake
        cmd = AckermannDriveStamped()
        MIN_TTC = 7

        if(ITTC[i]<MIN_TTC):
            self.drive_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





#NOTES:

        #Use ranges in LaserScan (all range messages radially ordered)
        #We will calculate the iTTC with LaserScan messages

        #Odometry - contains car's position, orientation and velocity
        #AckermannDriveStamped - set speed field to zero in order to stop the car. 
