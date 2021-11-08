#!/usr/bin/env python
# Author: Brett Kidman 2021
# import of relevant libraries.
import rospy # module for ROS APIs
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker

import math
import numpy as np
import tf
import time

""" ******* constants **************"""
OCCUPANCY_THRESHOLD = 50 #percent

""" ********* Main Robot Class ************************* """
class Robot_Camera:
    def __init__(self):
        """Initialization function."""
        # Setting up the publishers.
        self.marker_pub = rospy.Publisher("/markers", Marker, queue_size=10)
        self.pose_stamp_pub = rospy.Publisher("/pose_sequence", PoseStamped, queue_size=10)

        # only need to get grid data at initialization, dont need to subscribe
        grid_msg = rospy.wait_for_message("/map", OccupancyGrid, 3)
        self.grid = Grid(grid_msg.data, grid_msg.info.width, grid_msg.info.height, grid_msg.info.resolution)

        # Sleep important for allowing time for registration to the ROS master.
        rospy.sleep(2)

def main():
    """main function."""
    rospy.init_node("planner")
    plan = Planner()

    path = plan.astar_search((.25,.25),(6,8))
    print(path)
    plan.delete_markers() # clear markers from before
    plan.publish_markers(path)
    plan.publish_poses(path)


if __name__ == "__main__":
    """ run the main function. """
    main()