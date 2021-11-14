#!/usr/bin/env python

import numpy as np
import math

import rospy
import tf

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry


DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
#DEFAULT_SCAN_TOPIC_LASER = 'base_scan'
DEFAULT_SCAN_TOPIC_LASER = 'scan' #uncomment for real robot
DEFAULT_SCAN_TOPIC_ODOM = 'odom'
#DEFAULT_FRAME_LASER = 'base_laser_link'
DEFAULT_FRAME_LASER = 'laser' # uncomment for real robot

LASER_TIME = 1
FREQUENCY = 10
BUILD_TIME = 30


class Jacob:
    def __init__(self):
        self.read_map = None # the variable containing the map.
        self.new_map = None

        self.start_time = rospy.Time(0)

        self._map_sub = rospy.Subscriber("map", OccupancyGrid, self._map_callback, queue_size=1)

        # self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC_LASER, LaserScan, self._laser_callback, queue_size=1)

        self._odom_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC_ODOM, Odometry, self._odom_callback, queue_size=1)

        self.new_map_exists = False
        self.read_map_exists = False



    # map callback to store a map, resolution, height, and width
    def _map_callback(self, msg):
        print(rospy.get_rostime())
        print(self.start_time)
        print(rospy.Duration(BUILD_TIME))
        if rospy.get_rostime() - self.start_time< rospy.Duration(BUILD_TIME):
            self.read_map = np.reshape(msg.data, (msg.info.height, msg.info.width))
            print("read")
            self.read_map_exists = True
            print(self.read_map)
            self.resolution = msg.info.resolution

            self.height = msg.info.height
            self.width = msg.info.width
        else:
            self.new_map = np.reshape(msg.data, (msg.info.height, msg.info.width))
            self.new_map_exists = True
            print("new")
            print(self.new_map)
            self.resolution = msg.info.resolution

            self.height = msg.info.height
            self.width = msg.info.width



    def _odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.orientation = msg.pose.pose.orientation
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])


    def compare_grids(self):
        grid_diff = None
        grid_diff_exists = False
        if self.new_map_exists and self.read_map_exists:
            grid_diff = self.new_map - self.read_map
            grid_diff_exists = True
        if grid_diff_exists:
            for row in range(1, self.height-1):
                for column in range(1, self.width-1):
                    if grid_diff[row, column] > 0:
                        print("here")

                        negative_neighbors = 0

                        neighbors = []

                        neighbors.append(grid_diff[row+1, column])
                        neighbors.append(grid_diff[row-1, column])
                        neighbors.append(grid_diff[row+1, column+1])
                        neighbors.append(grid_diff[row-1, column+1])
                        neighbors.append(grid_diff[row+1, column-1])
                        neighbors.append(grid_diff[row-1, column-1])
                        neighbors.append(grid_diff[row, column+1])
                        neighbors.append(grid_diff[row, column-1])

                        for element in neighbors:
                            if element < 0:
                                negative_neighbors += 1

                        if negative_neighbors > 1:
                            print("imposter found")
                            # IMPOSTER!
            


    def spin(self):

        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            self.compare_grids()
            rate.sleep()


def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("robot")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    occ_object = Jacob()

    try:
        occ_object.spin()
        

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    """Run the main function."""
    main()