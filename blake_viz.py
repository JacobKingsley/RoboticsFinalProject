#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: October 10, 2021
# CS 81 Final Project
# Robot Movement Controller

# true division
from __future__ import division
from copy import deepcopy

# Import of python modules.
import math # use of pi.
import random
import numpy as np
from enum import Enum

from grid import Grid
from path_bfs import find_path, astar_search
from path_functions import get_grid_movements
from blake import Intruder_Patrol_Movement

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
import tf


# Constants.
# Time between message publishing to prevent skipping of messages
DT = .025

GRID_SQUARE_OCCUPIED = 100

# Visualization constants
SPHERE_SCALE = [.4, .4, .4]
SPHERE_SCALE_PATH = [.1, .1, .1]

RED = [1, 0, 0]
GREEN = [0, 1, 0]
BLUE = [0, 0, 1]

SPHERE = Marker.SPHERE
ARROW = Marker.ARROW

START = (0, -2.1)
GOAL = (1, 2)

AWAY_FROM_OBS = .5 # in m

ANGLE_THRESH = .04
LINEAR_THRESH = .03



# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"
DEFAULT_SCAN_TOPIC = "/scan" # name of topic for Stage simulator. For Gazebo, 'scan'
DEFAULT_ODOM_TOPIC = "/odom"
DEFAULT_GRID_TOPIC = "/map"


######	Turtlebot simulation constants
# constant holding the orientation of the front of the robot in the laser frame
LASER_FRONT = 0
# Field of view in radians that is checked in front of the robot for obstacles
MIN_SCAN_ANGLE_FRONT = 315 / 180 * math.pi
MAX_SCAN_ANGLE_FRONT = 45 / 180 * math.pi
HALF_FRONT_ANGLE = 45 / 180 * math.pi

# constant holding the orientation of the right of the robot in the laser frame
LASER_RIGHT = 3 * math.pi/2
LASER_LEFT = math.pi / 2


#####	Constants that are thesame for each
# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

DT_MODE = 15

# Velocities that will be used
LINEAR_VELOCITY = .6 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s




# minimum distance for registering obstacles in front of the robot
MIN_THRESHOLD_DISTANCE = .3 # m



START = (0, 0)



class Intruder_Patrol_Movement_Viz(Intruder_Patrol_Movement):
	def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
		"""Constructor."""

		# Setting up publishers/subscribers.
		self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)		# movement publisher
		self._occu_sub = rospy.Subscriber(DEFAULT_GRID_TOPIC, OccupancyGrid, self._occu_callback, queue_size=1)
		self._occu_pub = rospy.Publisher(DEFAULT_GRID_TOPIC, OccupancyGrid, queue_size=1)
		self._odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)
		
		self.linear_velocity = linear_velocity # Constant linear velocity set.
		self.angular_velocity = angular_velocity # Angular velocity based on LIDAR scan, but starts at 0

		self.marker_pub = rospy.Publisher("markers", Marker, queue_size=100)
		self.map = None # the variable containing the map.
		self.inflated_map = None

		self.markers_published = 0
		self.path = None
		self.grid_path = None

		self.x = START[0]
		self.y = START[1]
		self.yaw = 0

		self.next_waypoint = True


	
	def move(self, linear_vel, angular_vel):
		"""Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
		# Setting velocities.
		twist_msg = Twist()

		twist_msg.linear.x = linear_vel
		twist_msg.angular.z = angular_vel
		self._cmd_pub.publish(twist_msg)
		
	def _odom_callback(self, msg):

		self.x = msg.pose.pose.position.x + START[0]
		self.y = msg.pose.pose.position.y + START[1]

		print(msg.pose.pose.position.x, self.x, msg.pose.pose.position.y, self.y)
		self.orientation = msg.pose.pose.orientation
		(self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])

	def _occu_callback(self, msg):
		"""Processing of occupancy grid message."""

		# print(msg.header)
		# print("____")
		# print(msg.info)

		if not self.map:
			self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin)

			self.inflated_map = self.map.inflate_copy()

			# print("got map")
			self._occu_pub.publish(msg)

			self.publish_grid(msg, self.inflated_map)

	def publish_grid(self, original_msg, grid):
		occupancy_msg = original_msg

		occupancy_msg.data = grid.grid_as_list()

		self._occu_pub.publish(occupancy_msg)


	# Sets places start and goal markers on the vizualization
	def set_start_goal_map(self, start_x, start_y, goal_x, goal_y):
		self.publish_marker((start_x, start_y, 0), BLUE, SPHERE, SPHERE_SCALE)
		rospy.sleep(DT)
		self.publish_marker((goal_x, goal_y, 0), GREEN, SPHERE, SPHERE_SCALE)

	def publish_path_markers(self, path, grid=False):
		for point in path:
			if grid:
				point_i = self.map.grid_coords_to_m(point[0], point[1])
			else:
				point_i = point
			self.publish_marker(point_i, RED, SPHERE, SPHERE_SCALE_PATH)
			rospy.sleep(DT)

	def publish_marker(self, point, rgb, marker_type, scale):
		marker_msg = Marker()
		marker_msg.header.stamp = rospy.Time.now()
		marker_msg.header.frame_id = "map"
		
		marker_msg.action = Marker.ADD
		marker_msg.type = marker_type
		marker_msg.id = self.markers_published
		marker_msg.pose.position.x = point[0]
		marker_msg.pose.position.y = point[1]
		quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
		marker_msg.pose.orientation.x = quaternion[0]
		marker_msg.pose.orientation.y = quaternion[1]
		marker_msg.pose.orientation.z = quaternion[2]
		marker_msg.pose.orientation.w = quaternion[3]
		marker_msg.color.r = rgb[0]
		marker_msg.color.g = rgb[1]
		marker_msg.color.b = rgb[2]
		marker_msg.color.a = 1
		marker_msg.scale.x = scale[0]
		marker_msg.scale.y = scale[1]
		marker_msg.scale.z = scale[2]
		self.marker_pub.publish(marker_msg)

		# increment markers counter
		self.markers_published += 1


	def go_to_waypoint(self, waypoint):
		# Uses a BFS to find the shortest path between the start and end points in the map frame
		# (self.path, self.grid_path) = find_path(self.inflated_map, START, GOAL)

		start_grid = self.inflated_map.m_to_grid_coords(self.x, self.y)
		goal_grid = waypoint

		# self.path = astar_search(self.inflated_map, start_grid, goal_grid)
		self.grid_path = find_path(self.inflated_map, start_grid, goal_grid)

		# if path is an empty list, it means a path was not found
		# Either there is no path, or the start or end positions are on occupied spaces (walls)
		if self.path == []:
			print("Error: no path found. Either no path exists or the start or end positions are on occupied spaces")
			self.path = []
			# return

		# get movements from list of points and travel along path
		if self.grid_path:
			self.publish_path_markers(self.grid_path, True)
			grid_movements = get_grid_movements(self.grid_path)
			self.move_along_path(grid_movements)



	def spin(self):
		rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
		while not rospy.is_shutdown() and self.next_waypoint:
			# run only when the map is loaded
			if self.inflated_map:
				start_pt = self.inflated_map.m_to_grid_coords(self.x, self.y)

				if not self.inflated_map.is_valid(start_pt[0], start_pt[1]):
					print("ERROR: START NOT VALID")

				waypoint = self.generate_waypoint()


				waypoint_m = self.inflated_map.grid_coords_to_m(waypoint[0], waypoint[1])

				print((self.x, self.y, waypoint_m[0], waypoint_m[1]))
				self.set_start_goal_map(self.x, self.y, waypoint_m[0], waypoint_m[1])

				self.next_waypoint = False

				self.go_to_waypoint(waypoint)

				
			
		rate.sleep()
		

def main():
	"""Main function."""

	# 1st. initialization of node.
	rospy.init_node("intruder_patrol_movement_viz")

	# Sleep for a few seconds to wait for the registration.
	rospy.sleep(2)

	# Initialization of the class for the mobile motion.
	intruder_patrol_movement_viz = Intruder_Patrol_Movement_Viz()

	# If interrupted, send a stop command before interrupting.
	rospy.on_shutdown(intruder_patrol_movement_viz.stop)

	# Robot mobile motion.
	try:
		intruder_patrol_movement_viz.spin()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
	"""Run the main function."""
	main()
