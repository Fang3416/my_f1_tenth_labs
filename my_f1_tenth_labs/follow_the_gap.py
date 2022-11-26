import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        self.subscriber_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # TODO: Publish to drive
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        #parameters that we choose:
        self.min_distance = 5.0 #check this
        self.driving_speed = 2.0 #check this

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        #need to work out what to do here
        #maybe get rid of all values greater than 3?

        proc_ranges = ranges
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_start_index = 0
        max_end_index = 0
        current_start_index = 0
        current_end_index = 0

        size_of_max_gap = 0
        size_of_current_gap = 0

        for i in range(len(free_space_ranges)):
            if(free_space_ranges[i]>self.min_distance):
                size_of_current_gap = size_of_current_gap+1
                current_end_index = i
                if(size_of_current_gap > size_of_max_gap):
                    size_of_max_gap = size_of_current_gap
                    max_start_index = current_start_index
                    max_end_index = current_end_index
            else:
                size_of_current_gap = 0
                current_start_index = i+1                   

        return (max_start_index, max_end_index)
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        current_furthest_point = ranges[start_i]
        index_furthest_point = start_i
        for i in range((end_i-start_i)+1):
            if(ranges[start_i+i]>current_furthest_point):
                current_furthest_point = ranges[start_i+i]
                index_furthest_point = start_i+i

        return index_furthest_point

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR
        closest_point = ranges[0]
        closest_point_index = 0
        for i in range(len(ranges)):
            if(ranges[i]<closest_point):
                closest_point = ranges[i]
                closest_point_index = i

        #Eliminate all points inside 'bubble' (set them to zero)
        #modify this to work for a specific radius
        #Use trig to fix up
        if(closest_point_index == 0):
            ranges[closest_point_index+1] = 0
        elif(closest_point_index == (len(ranges)-1)):
            ranges[closest_point_index-1] = 0
        else:
            ranges[closest_point_index-1] = 0
            ranges[closest_point_index+1] = 0

        ranges[closest_point_index] = 0

        #Find max length gap 
        indices_tuple = self.find_max_gap(ranges)
        print(indices_tuple) #(420,619)
        #print(len(ranges)) 1080

        #Find the best point in the gap 
        best_point_index = self.find_best_point(indices_tuple[0], indices_tuple[1], ranges)
        #print(best_point_index)

        #Publish Drive message
        drive_msg = AckermannDriveStamped()

        steering_angle = data.angle_min+(best_point_index*data.angle_increment)
        #print(ranges[indices_tuple[0]:indices_tuple[1]])
        print("The best_point index: ", best_point_index)
        print("The best point value is ", ranges[best_point_index])
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.driving_speed
        
        self.drive_publisher.publish(drive_msg)
        #print(max(ranges)) 30

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
