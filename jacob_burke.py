#!/usr/bin/env python

import numpy as np
import math

import rospy
import tf

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry # message type for odom



DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
#DEFAULT_SCAN_TOPIC_LASER = 'base_scan'
DEFAULT_SCAN_TOPIC_LASER = 'scan' #uncomment for real robot
DEFAULT_SCAN_TOPIC_ODOM = 'odom'
#DEFAULT_FRAME_LASER = 'base_laser_link'
DEFAULT_FRAME_LASER = 'laser' # uncomment for real robot

LASER_TIME = 1

class Jacob:
    def __init__(self):
        self.read_map = None # the variable containing the map.
        self.built_map = None

        self.sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)

        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC_LASER, LaserScan, self._laser_callback, queue_size=1)

        self._odom_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC_ODOM, Odometry, self._odom_callback, queue_size=1)



    # map callback to store a map, resolution, height, and width
    def map_callback(self, msg):
        self.read_map = np.reshape(msg.data, (msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution

        self.height = msg.info.height
        self.width = msg.info.width

    def _odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.orientation = msg.pose.pose.orientation
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])

    def _laser_callback(self, msg):
        # check that it has been a second and then reset self.time
        if (msg.header.stamp - self.time).to_sec() < LASER_TIME:
            return

        self.time = msg.header.stamp

        # transformation matrix for both translation and rotation between the odom and map frames
        self.transform_listener.waitForTransform(DEFAULT_SCAN_TOPIC_ODOM, DEFAULT_FRAME_LASER, self.time, rospy.Duration(4.0))
        (translation, rotation) = self.transform_listener.lookupTransform(DEFAULT_SCAN_TOPIC_ODOM, DEFAULT_FRAME_LASER, self.time)
        t = tf.transformations.translation_matrix(translation)
        r = tf.transformations.quaternion_matrix(rotation)

        full_transformation = t.dot(r)

        # set the robot's odom x and y
        robot_odom = full_transformation.dot(np.array([[0], [0], [0], [1]]))
        robot_odom_x = int(round(robot_odom[0][0] / self.resolution))
        robot_odom_y = int(round(robot_odom[1][0] / self.resolution))


        # iterate through all the laser scans
        for i, dist in enumerate(msg.ranges):
            dist = min(dist,msg.range_max)
            # find the current angle
            current_a = msg.angle_min + (i * msg.angle_increment)

            x_base = dist * math.cos(current_a)
            y_base = dist * math.sin(current_a)

            # scanned x and y
            robot_scan = full_transformation.dot(np.array([[x_base], [y_base], [0], [1]]))
            scan_x = int(round(robot_scan[0][0] / self.resolution))
            scan_y = int(round(robot_scan[1][0] / self.resolution))
        
            if dist < msg.range_max: # if we hit a wall
                self.bresenham(self.grid, robot_odom_x, robot_odom_y, scan_x, scan_y, hit_wall=True)
            else: # if we don't hit a wall
                self.bresenham(self.grid, robot_odom_x, robot_odom_y, scan_x, scan_y, hit_wall=False)
            

    def bresenham(self, grid, x1, y1, x2, y2, wall=True):
        # output: (list empty, tuple occupied)
        # 
        # 	empty:
        # 		A list of tuples containing (int x, int y) representing
        # 		coordinate points in thr grid frame of the points along
        # 		the ray that are unoccupied
        # 
        # 	occupied:
        # 		A tuple formatted (int x, int y) holding the coordinate
        # 		at the end of the ray in grid coordinates that is either
        # 		the point on the object detected or is located a distance
        # 		away equal to the max range of the sensor. In the former
        # 		case the point is occupied, in the latter case the
        # 		point is unoccupied
        #
        # takes the grid to be edited, the start and end points of a line
        # segment in grid square coordinates, and whether the point (x2, y2)
        # is a wall (occupied point) or empty
        dx = x2 - x1
        dy = y2 - y1

        # if dx = 0, its a vertical line and the points need to be incremented along the y coordinates
        # if |m| > 1 (or |m| is infinity), swap coordinates, compute, and then swap back
        # so that algorithm can increment along the y-coordinates instead of the x
        if dx == 0 or abs(dy / dx) > 1:
            # run algorithm with swapped x and y for |m| > 1
            (empty, occupied) = self.bresenham_helper(y1, x1, y2, x2)
            # swap coordinates back
            for i in range(len(empty)):
                empty[i] = (empty[i][1], empty[i][0])
            occupied = (occupied[1], occupied[0])
            
        else:	# if |m| <= 1, run normally
            (empty, occupied) = self.bresenham_helper(x1, y1, x2, y2)

        # now that we have the points along the line segment, set their occupancy value as appropriate
        # 0 for empty spaces, 100 for occupied spaces
        for pt in empty:
            # TODO: ADD POINT TO GRID
            pass
            # grid.set_point(pt[0], pt[1], 0)

        # if no_wall flag is on, then the laser detected max distance
        # so point at the end of the ray should be unoccupied
        # TODO: ADD END OF RAY POINT TO GRID AS EITHER OCCUPIED OR EMPTY
        if wall:
            pass
            # grid.set_point(occupied[0], occupied[1], 100)
        else:
            pass
            # grid.set_point(occupied[0], occupied[1], 0)

        return empty, occupied


    def bresenham_helper(self, x1, y1, x2, y2):
        # helper function for bresenham function that takes in start point (x1, y1) and end point (x2, y2), and the slope m
        # 
        # output: (list empty, tuple occupied)
        # 
        # 	empty:
        # 		A list of tuples containing (int x, int y) representing
        # 		coordinate points in thr grid frame of the points along
        # 		the ray that are unoccupied
        # 
        # 	occupied:
        # 		A tuple formatted (int x, int y) holding the coordinate
        # 		at the end of the ray in grid coordinates that is either
        # 		the point on the object detected or is located a distance
        # 		away equal to the max range of the sensor. In the former
        # 		case the point is occupied, in the latter case the
        # 		point is unoccupied
        #

        m = (y2 - y1) / (x2 - x1)

        # list holding output points along line segment that are not the endpoint
        empty = []

        # holds cumulative error as we increment
        e = 0

        # start point for the y value
        y = y1

        # find directions that x and y increment in
        if x2 > x1:
            ddx = 1
        else:
            ddx = -1

        if y2 > y1:
            ddy = 1
        else:
            ddy = -1

        for x in range(x1, x2+ddx, ddx):
            # add to the list of empty points
            empty.append((x, y))
            if (e + (m*ddx)) * ddy < 0.5:
                e += (m*ddx)
            else:
                e += (m*ddx) - ddy
                y += ddy

        # last point is an occupied point
        occupied = (x2, y2)

        return empty, occupied
