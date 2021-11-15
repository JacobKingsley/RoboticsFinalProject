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

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry, OccupancyGrid
import tf


# Constants.
GRID_SQUARE_OCCUPIED = 100


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


# Velocities that will be used
LINEAR_VELOCITY = .25 # m/s
ANGULAR_VELOCITY = math.pi/6 # rad/s


# minimum distance for registering obstacles in front of the robot
MIN_THRESHOLD_DISTANCE = .3 # m


START = (0, 0)

class Intruder_Patrol_Movement():
	def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY,
	min_threshold_distance=MIN_THRESHOLD_DISTANCE, front_scan_angle=[MIN_SCAN_ANGLE_FRONT, MAX_SCAN_ANGLE_FRONT]):
		"""Constructor."""

		# Setting up publishers/subscribers.
		self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=100)		# movement publisher
		self._occu_sub = rospy.Subscriber(DEFAULT_GRID_TOPIC, OccupancyGrid, self._occu_callback, queue_size=1)
		self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)	# laser subscriber
		self._odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)
		
		self.linear_velocity = linear_velocity # Constant linear velocity set.
		self.angular_velocity = angular_velocity # Angular velocity based on LIDAR scan, but starts at 0

		self.map = None # the variable containing the map.
		self.inflated_map = None

		self.path = None
		self.grid_path = None

		self.x = 0
		self.y = 0
		self.yaw = 0

		self.next_waypoint = True
		self.front_scan_angle = front_scan_angle

		self.front_obs = False
		self.min_threshold_distance = min_threshold_distance


	def move(self, linear_vel, angular_vel):
		"""Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
		# Setting velocities.
		twist_msg = Twist()

		twist_msg.linear.x = linear_vel
		twist_msg.angular.z = angular_vel
		self._cmd_pub.publish(twist_msg)

	def _laser_callback(self, msg):
		forward_i = int(round((LASER_FRONT - msg.angle_min) / msg.angle_increment))
		forward_i_2 = int(round(((LASER_FRONT + 2 * math.pi) - msg.angle_min) / msg.angle_increment))
		self.forward_scan_distance = msg.ranges[forward_i]

		# print(len(msg.ranges))
		# print(msg.angle_min)
		# print(msg.angle_max)
		# print(msg.angle_increment)


		# check front and set a flag if the robot gets too close to an obstacle
		f_start_index = int(round((self.front_scan_angle[0] - msg.angle_min) / msg.angle_increment))
		f_end_index = int(round((self.front_scan_angle[1] - msg.angle_min) / msg.angle_increment))

		ranges_spliced = msg.ranges[f_start_index : forward_i_2] + msg.ranges[forward_i : f_end_index + 1]

		f_min_value = np.min(ranges_spliced)

		# print(f_min_value)
		
		if f_min_value < self.min_threshold_distance:
			self.front_obs = True
			self.reverse
		

	def _odom_callback(self, msg):
		self.x = msg.pose.pose.position.x + START[0]
		self.y = msg.pose.pose.position.y + START[1]
		self.orientation = msg.pose.pose.orientation
		(self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])

	def _occu_callback(self, msg):
		"""Processing of occupancy grid message."""

		if not self.map:
			self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin)

			# makes a copy of the map with inflated obstacles
			self.inflated_map = self.map.inflate_copy()


	def stop(self):
		print("stopping")
		"""Stop the robot."""
		twist_msg = Twist()
		self._cmd_pub.publish(twist_msg)

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


			# """old method: using only timer"""
			# # Get current time.
			# start_time = rospy.get_rostime()
			# # publish rotation until enough time elapses so the robot turns the given angle
			# while rospy.get_rostime() - start_time < rospy.Duration(abs(angle) / self.angular_velocity):
			# 	self.move(0, ang_vel)

			"""new method: calculating destination orientation using odom"""
			# calculate the desired final orientation of the robot
			# puts the angle in the range (-pi, pi)
			dest_yaw = (self.yaw + angle) % (2 * math.pi)
			if dest_yaw >= math.pi:
				dest_yaw -= 2 * math.pi

			# publish rotation until current yaw is the desired yaw.
			while abs(self.calc_angle_distance(self.yaw, dest_yaw)) > ANGLE_THRESH:
				self.move(0, ang_vel)

				"""print statements for testing"""
				# print("dist: " + str(self.calc_angle_distance(self_yaw, dest_yaw)))
				# print("current: " + str(self_yaw) + ",  dest: " + str(dest_yaw))
			
				rate.sleep()

			self.stop()

	def drive(self, distance):
		rate = rospy.Rate(FREQUENCY)

		if not rospy.is_shutdown():

			# if distance is negative, move backwards
			if distance < 0:
				lin_vel = - self.linear_velocity
			else: # if distance is positive, move forward
				lin_vel = self.linear_velocity

			# """old method: using only timer"""
			# # Get current time.
			# start_time = rospy.get_rostime()
			# # publish motion until enough time elapses so the robot moves the given distance
			# while rospy.get_rostime() - start_time < rospy.Duration(abs(distance) / self.linear_velocity):
			# 	self.move(lin_vel, 0)

			"""new method: calculating destination and using odom"""
			# calculate the desired final poistion of the robot
			dest_x = self.x + (distance * math.cos(self.yaw))
			dest_y = self.y + (distance * math.sin(self.yaw))
			
			# publish movement until the position of the robot (odom) is the desired position
			while self.distance_between(self.x, self.y, dest_x, dest_y) > LINEAR_THRESH:
				self.move(lin_vel, 0)

				"""print statements for testing"""
				# print("dist: " + str(math.sqrt(((self.x - dest_x)**2) + ((self.y - dest_y)**2))))
				# print("current: " + str((self.x, self.y)) + ",  dest: " + str((dest_x, dest_y)))

				rate.sleep()

			self.stop()

	def reverse(self):
		print("rev")

		
		self.drive(-.2)
		self.stop()
		self.rotate(math.pi)
		self.stop()

		curr_grid = self.inflated_map.m_to_grid_coords(self.x, self.y)
		while not self.inflated_map.is_valid(curr_grid[0], curr_grid[1]):
			self.drive(.2)
			curr_grid = self.inflated_map.m_to_grid_coords(self.x, self.y)

		curr_grid = self.inflated_map.m_to_grid_coords(self.x, self.y)
		if self.inflated_map.is_valid(curr_grid[0], curr_grid[1]):
			self.stop()


	def rotate_to(self, orientation_angle):
		# rotation to given orientation in the odom frame
		d_angle = (orientation_angle - self.yaw) % (2 * math.pi)

		# see which way to rotate (shortest way)
		if d_angle < math.pi:	# rotate counterclockwise
			self.rotate(d_angle)
		else: # rotate clockwise
			self.rotate(d_angle - (2 * math.pi))

	def go_to_point(self, x, y):
		# rotation, translation, method to get robot to a certain position
		dest_angle = math.atan2(y - self.y, x - self.x)
		# find distance the robot has to go to that point
		dest_distance = self.distance_between(self.x, self.y, x, y)
		# find rotation angle for final orientation

		self.rotate_to(dest_angle)	# rotate toward destination point
		self.stop()

		self.drive(dest_distance)	# drive calculated distance
		self.stop()


	def move_along_path(self, movements):

		# for movement in movements:
		# 	if movement[0] == 'x' and movement[1] > 0:
		# 		self.rotate_to(0)
		# 	elif movement[0] == 'x' and movement[1] < 0:
		# 		self.rotate_to(math.pi)

		# 	if movement[0] == 'y' and movement[1] > 0:
		# 		self.rotate_to(math.pi / 2)
		# 	elif movement[0] == 'y' and movement[1] < 0:
		# 		self.rotate_to(-math.pi / 2)

			# d = movement[1] * self.map.resolution

		# 	self.drive(d)

		for movement in movements:
			d = movement[1] * self.map.resolution

			if movement[0] == 'x':
				self.go_to_point(self.x + d, self.y)
			elif movement[0] == 'y':
				self.go_to_point(self.x, self.y + d)

			
		rospy.sleep(1)
		self.next_waypoint = True
	
	def generate_waypoint(self):
		width = self.inflated_map.width
		height = self.inflated_map.height

		rand_x = int(round(width * random.random()))
		rand_y = int(round(height * random.random()))

		while not self.inflated_map.is_valid(rand_x, rand_y):
			rand_x = int(round(width * random.random()))
			rand_y = int(round(height * random.random()))

		return (rand_x, rand_y)


	def go_to_waypoint(self, waypoint):
		# Uses a BFS to find the shortest path between the start and end points in the map frame
		# (self.path, self.grid_path) = find_path(self.inflated_map, START, GOAL)

		start_grid = self.inflated_map.m_to_grid_coords(self.x, self.y)
		goal_grid = waypoint

		# self.grid_path = astar_search(self.inflated_map, start_grid, goal_grid)
		self.grid_path = find_path(self.inflated_map, start_grid, goal_grid)

		# if path is an empty list, it means a path was not found
		# Either there is no path, or the start or end positions are on occupied spaces (walls)
		if self.grid_path == []:
			print("Error: no path found. Either no path exists or the start or end positions are on occupied spaces")
			self.grid_path = []
			# return

		# get movements from list of points and travel along path
		if self.grid_path:
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
					self.reverse()

					continue

				waypoint = self.generate_waypoint()

				self.next_waypoint = False

				print(start_pt, waypoint)

				self.go_to_waypoint(waypoint)

				
			
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
