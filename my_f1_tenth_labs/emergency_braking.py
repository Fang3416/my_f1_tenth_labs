#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
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
        self.speed = 0.0
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min # [rad]
        angle_inc = scan_msg.angle_increment # [rad]
        num_steps = len(ranges)
        angles = [angle_min + i*angle_inc for i in range(num_steps)]
        v = -self.speed
        r_dot = [v*np.cos(a) for a in angles]

        TTC_list = [np.inf for _ in range(num_steps)]
        for i in range(num_steps):
            if max([-r_dot[i], 0]) == 0 or np.isnan(ranges[i]) or angles[i]<-0.3 or angles[i]>0.3:
                continue
            else:
                TTC_list[i] = ranges[i] / max([-r_dot[i], 0])
        TTC = min(TTC_list)

        if TTC < 2:
            print('AEB Activated')
            print(f"TTC Measured: {TTC:0.3f}\n")
            self.speed = 0.0
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = self.speed
            self.publisher.publish(ack_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
