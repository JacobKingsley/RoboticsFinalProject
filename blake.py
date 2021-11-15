#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: October 10, 2021
# CS 81 Final Project
# Robot Movement Controller

# true division
from __future__ import division

# Import of python modules.
import math # use of pi.
import random
import numpy as np
from enum import Enum

from grid import Grid
from path_bfs import find_path
from path_functions import get_grid_movements

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



class Intruder_Patrol_Movement():
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

		self.x = 0
		self.y = 0
		self.yaw = 0


	
	def move(self, linear_vel, angular_vel):
		"""Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
		# Setting velocities.
		twist_msg = Twist()

		twist_msg.linear.x = linear_vel
		twist_msg.angular.z = angular_vel
		self._cmd_pub.publish(twist_msg)
		
	def _odom_callback(self, msg):
		
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
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

	def publish_path_markers(self, path):
		for point in path:
			self.publish_marker(point, RED, SPHERE, SPHERE_SCALE_PATH)
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

	def stop(self):
		print("stopping")
		"""Stop the robot."""
		twist_msg = Twist()
		self._cmd_pub.publish(twist_msg)
	

	# def rotate(self, angle):
	# 	rate = rospy.Rate(FREQUENCY)

	# 	if not rospy.is_shutdown():

	# 		# if angle is negative, rotate clockwise (negative angular velocity)
	# 		if angle < 0:
	# 			ang_vel = - self.angular_velocity
	# 		else: # if angle is positive, rotate counterclockwise (positive angular velocity)
	# 			ang_vel = self.angular_velocity

						
	# 		"""using only timer"""
	# 		# Get current time.
	# 		start_time = rospy.get_rostime()
	# 		# publish rotation until enough time elapses so the robot turns the given angle
	# 		while rospy.get_rostime() - start_time < rospy.Duration(abs(angle) / self.angular_velocity):
	# 			self.move(0, ang_vel)
			
	# 			rate.sleep()

	def calc_angle_distance(self, angle_1, angle_2):
		# calculates the distance between two angles.
		d_angle = (angle_2 - angle_1) % (2 * math.pi)

		# makes sure output is in range (-pi, pi)
		# this way sign of output indicates quickest direction to rotate from angle_1 to angle_2
		if d_angle >= math.pi:
			d_angle -= 2 * math.pi

		return d_angle

	def distance_between(self, x1, y1, x2, y2):
		# calculates euclidean distance between two points
		return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

	def rotate(self, angle):
		rate = rospy.Rate(FREQUENCY)

		if not rospy.is_shutdown():

			# if angle is negative, rotate clockwise (negative angular velocity)
			if angle < 0:
				ang_vel = - self.angular_velocity
			else: # if angle is positive, rotate counterclockwise (positive angular velocity)
				ang_vel = self.angular_velocity

			"""new method: calculating destination orientation using odom"""
			# calculate the desired final orientation of the robot
			# puts the angle in the range (-pi, pi)
			dest_yaw = (self.odom_yaw + angle) % (2 * math.pi)
			if dest_yaw >= math.pi:
				dest_yaw -= 2 * math.pi

			# publish rotation until current yaw is the desired yaw.
			while abs(self.calc_angle_distance(self.odom_yaw, dest_yaw)) > self.angle_thresh:
				self.move(0, ang_vel)

				"""print statements for testing"""
				# print("dist: " + str(self.calc_angle_distance(self.odom_yaw, dest_yaw)))
				# print("current: " + str(self.odom_yaw) + ",  dest: " + str(dest_yaw))
			
				rate.sleep()

	def drive(self, distance):
		rate = rospy.Rate(FREQUENCY)

		if not rospy.is_shutdown():

			# if distance is negative, move backwards
			if distance < 0:
				lin_vel = - self.linear_velocity
			else: # if distance is positive, move forward
				lin_vel = self.linear_velocity

			"""new method: calculating destination and using odom"""
			# calculate the desired final poistion of the robot
			dest_x = self.odom_x + (distance * math.cos(self.odom_yaw))
			dest_y = self.odom_y + (distance * math.sin(self.odom_yaw))
			
			# publish movement until the position of the robot (odom) is the desired position
			while self.distance_between(self.odom_x, self.odom_y, dest_x, dest_y) > self.linear_thresh:
				self.move(lin_vel, 0)

				"""print statements for testing"""
				# print("dist: " + str(math.sqrt(((self.odom_x - dest_x)**2) + ((self.odom_y - dest_y)**2))))
				# print("current: " + str((self.odom_x, self.odom_y)) + ",  dest: " + str((dest_x, dest_y)))

				rate.sleep()


	def polyline(self, world_points):

		# translate points to odom frame so they work with self.go_to_point()
		translated_pts_odom = self.world_to_odom(world_points)

		for pt in translated_pts_odom:
			self.go_to_point(pt[0], pt[1])

		# go back to first point in the list
		self.go_to_point(translated_pts_odom[0][0], translated_pts_odom[0][1])


	def rotate_to(self, orientation_angle):
		# rotation to given orientation in the odom frame
		d_angle = (orientation_angle - self.odom_yaw) % (2 * math.pi)

		# see which way to rotate (shortest way)
		if d_angle < math.pi:	# rotate counterclockwise
			self.rotate(d_angle)
		else: # rotate clockwise
			self.rotate(d_angle - (2 * math.pi))

	def go_to_point(self, x, y):
		# rotation, translation, method to get robot to a certain position
		dest_angle = math.atan2(y - self.odom_y, x - self.odom_x)
		# find distance the robot has to go to that point
		dest_distance = self.distance_between(self.odom_x, self.odom_y, x, y)
		# find rotation angle for final orientation

		self.rotate_to(dest_angle)	# rotate toward destination point
		self.drive(dest_distance)	# drive calculated distance


	def go_to_state(self, x, y, theta):
		# rotation, translation, rotation method to get robot to a certain position and orientation
		self.go_to_point(x, y)	# drive to point
		self.rotate_to(theta)	# rotate to given final orientation


	def spin(self):
		rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
		while not rospy.is_shutdown():
			# run only when the map is loaded
			if self.inflated_map:

				# visualize start and end points
				print("start goal points")
				self.set_start_goal_map(START[0], START[1], GOAL[0], GOAL[1])

				# Uses a BFS to find the shortest path between the start and end points in the map frame
				(self.path, self.grid_path) = find_path(self.inflated_map, START, GOAL)

				# if path is an empty list, it means a path was not found
				# Either there is no path, or the start or end positions are on occupied spaces (walls)
				if self.path == []:
					print("Error: no path found. Either no path exists or the start or end positions are on occupied spaces")
					self.path = []
					# return

			# publish the start and end markers again in case the
			# vizualization reset in the time it takes the BFS to run
			self.set_start_goal_map(START[0], START[1], GOAL[0], GOAL[1])

			# publish each pose as a PoseStamped message and as a marker for the vizualization
			if self.path:
				self.publish_path_markers(self.path)
				grid_movements = get_grid_movements(self.grid_path)

				
			
		rate.sleep()
		

def main():
	"""Main function."""

	# 1st. initialization of node.
	rospy.init_node("intruder_patrol_movement")

	# Sleep for a few seconds to wait for the registration.
	rospy.sleep(2)

	# Initialization of the class for the mobile motion.
	intruder_patrol_movement = Intruder_Patrol_Movement()

	# If interrupted, send a stop command before interrupting.
	rospy.on_shutdown(intruder_patrol_movement.stop)

	# Robot mobile motion.
	try:
		intruder_patrol_movement.spin()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
	"""Run the main function."""
	main()
