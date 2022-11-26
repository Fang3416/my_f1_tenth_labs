import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from math import sin, cos, isfinite
import time

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        #parameters we can manipulate:
        self.look_ahead_distance = 1
        self.desired_distance = 1

        # TODO: create subscribers and publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriber_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # TODO: set PID gains
        self.kp = 3
        self.kd = 2
        self.ki = 1

        # TODO: store history
        self.integral = 0 
        self.prev_error = 0 
        self.error = 0

        # TODO: store any necessary values you think you'll need
        self.min_angle = 0
        self.max_angle = 0
        self.angle_increment = 0
        self.time_current = 0
        self.time_previous = 0

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.
        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR
        Returns:
            range: range measurement in meters at the given angle
        """

        #TODO: implement
        
        index = int(np.floor((angle-self.min_angle)/self.angle_increment))

        try:
            return range_data[index]

        except IndexError as e:
            print(f"{e}")
            print(f"length of dists = {len(range_data)}, and you're asking for index {index}!")

        return None

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()
        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall
        Returns:
            error: calculated error
        """
        #TODO:implement

        angle_b = np.pi/2
        angle_a = np.pi/4

        while (not isfinite(self.get_range(range_data, angle_a))) or (angle_a == angle_b): #added this in in case of errors
            angle_a += 0.01

        while (not isfinite(self.get_range(range_data, angle_b))) or (angle_a == angle_b):
            angle_b += 0.01

        distance_a = self.get_range(range_data, angle_a)   #pi/4 radians from beam b
        distance_b = self.get_range(range_data, angle_b)  #pi/2 radians from x axis
        theta = -(angle_a-angle_b) #angle between beams a and b
        alpha = np.arctan2((distance_a*np.cos(theta)-distance_b),(distance_a*np.sin(theta))) #arctan2?
        
        L= self.look_ahead_distance

        actual_distance = distance_b*np.cos(alpha)+L*np.sin(alpha)

        #update all important variables here:
        error = dist-actual_distance
        self.prev_error = self.error
        self.error = error
        self.time_previous = self.time_current
        self.time_current = time.time()
        time_difference = self.time_current - self.time_previous
        self.integral = self.prev_error*(time_difference)

        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control
        Args:
            error: calculated error
            velocity: desired velocity
        Returns:
            None
        """
        # TODO: Use kp, ki & kd to implement a PID controller
        steering_angle = -(self.kp*error+self.ki*self.integral+self.kd*((error-self.prev_error)/(self.time_current-self.time_previous)))
        #why minus?

        # TODO: fill in drive message and publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        
        if(abs(np.rad2deg(steering_angle))<10):
            drive_msg.drive.speed = 1.5
        elif(abs(np.rad2deg(steering_angle))>10 and abs(np.rad2deg(steering_angle))<20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5

        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.
        Args:
            msg: Incoming LaserScan message
        Returns:
            None
        """
        self.min_angle = msg.angle_min
        self.max_angle = msg.angle_max
        self.angle_increment = msg.angle_increment

        error = self.get_error(msg.ranges, self.desired_distance) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
