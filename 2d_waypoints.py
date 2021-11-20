#!/usr/bin/env python

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: November 15, 2021
# CS 81 Final Project
# Robot Movement Controller 2D Simulation -- Waypoint Method

# true division
from __future__ import division

# Import of python modules.
import math # use of pi.
from enum import Enum

from grid import Grid
from path_functions import get_grid_movements, generate_waypoint, find_path

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry, OccupancyGrid
import tf


# Constants.

ANGLE_THRESH = .04
LINEAR_THRESH = .03



# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"
DEFAULT_ODOM_TOPIC = "/odom"
DEFAULT_GRID_TOPIC = "/map"

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used
LINEAR_VELOCITY = .4 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

# small distance used in WaypointPatrol2D.unstick_wall()
DX = .2

##### Set the point in the map (in m) that the robot starts out at #####
# Assumes the robot starts out at an orientation angle of 0
START = (2, 2)

class FSM(Enum):
	GENERATE_POINTS = 1
	FOLLOW_PATH = 2
	UNSTICK_WALL = 3
	STOP = 4

class WaypointPatrol2D():
	def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
		"""Constructor."""

		# Setting up publishers/subscribers.
		self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=100)		# movement publisher
		self._occu_sub = rospy.Subscriber(DEFAULT_GRID_TOPIC, OccupancyGrid, self._occu_callback, queue_size=1)
		self._odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)
		
		self.linear_velocity = linear_velocity # Constant linear velocity set.
		self.angular_velocity = angular_velocity # Angular velocity based on LIDAR scan, but starts at 0

		self.map = None # the variable containing the map.
		self.inflated_map = None # the variable containing the map with inflated edges and occupied points

		self.grid_path = None	# list of points along the found path
		self.grid_movements = None	# list of movements the robot has to do to move along the path

		# set initial odometry
		self.x = 0
		self.y = 0
		self.yaw = 0

		# set initial FSM state
		self._fsm = FSM.GENERATE_POINTS


	def move(self, linear_vel, angular_vel):
		"""Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
		# Setting velocities.
		twist_msg = Twist()

		twist_msg.linear.x = linear_vel
		twist_msg.angular.z = angular_vel
		self._cmd_pub.publish(twist_msg)
		

	def _odom_callback(self, msg):
		# set odom points based on initial START point where the robot begins
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

			# calculate the desired final orientation of the robot
			# rotate to that orientation using odometry
			# puts the angle in the range (-pi, pi)
			dest_yaw = (self.yaw + angle) % (2 * math.pi)
			if dest_yaw >= math.pi:
				dest_yaw -= 2 * math.pi

			# publish rotation until current yaw is the desired yaw.
			while abs(self.calc_angle_distance(self.yaw, dest_yaw)) > ANGLE_THRESH:
				self.move(0, ang_vel)

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

			# drive the desired distance by finding how far the robot has travelled using odometry
			original_x = self.x
			original_y = self.y

			# move until the robot has travelled the desired amount
			while abs(self.distance_between(self.x, self.y, original_x, original_y) - distance) > LINEAR_THRESH:
				self.move(lin_vel, 0)

				rate.sleep()

			self.stop()

	def unstick_wall(self):
		# method that reverses the robot until it moves to an unoccupied space and then resets
		self.stop()

		curr_grid = self.inflated_map.m_to_grid_coords(self.x, self.y)
		while not self.inflated_map.is_valid(curr_grid[0], curr_grid[1]):
			self.drive(-DX)
			curr_grid = self.inflated_map.m_to_grid_coords(self.x, self.y)

		self.stop()

		self._fsm = FSM.GENERATE_POINTS


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

		for movement in movements:

			# get out of function if not in this state
			if self._fsm != FSM.FOLLOW_PATH:
				return
			
			d = movement[1] * self.map.resolution

			if movement[0] == 'x':
				self.go_to_point(self.x + d, self.y)
			elif movement[0] == 'y':
				self.go_to_point(self.x, self.y + d)

			rospy.sleep(.2)

		rospy.sleep(1)
		self._fsm = FSM.GENERATE_POINTS


	# generates the random waypoints and find the path to that waypoint from the robot's current pose
	def generate_path(self):
		
		# find grid coordinate of current location
		start_pt = self.inflated_map.m_to_grid_coords(self.x, self.y)
		
		
		# check that the current start position is a valid unoccupied point on the map
		if not self.inflated_map.is_valid(start_pt[0], start_pt[1]):
			print("ERROR: START NOT VALID")
			self._fsm = FSM.UNSTICK_WALL
			return

		# keep generating new random waypoints until one is valid (an unoccupied space in the map)
		waypoint = generate_waypoint(self.inflated_map)

		start_grid = start_pt
		goal_grid = waypoint

		# find the path from the current point to the waypoint
		# Uses a BFS to find the shortest path between the start and end points in the grid framE
		self.grid_path = find_path(self.inflated_map, start_grid, goal_grid)

		
		# if path is an empty list, it means a path was not found
		# Either there is no path, or the start or end positions are on occupied spaces (walls)
		if self.grid_path == []:
			print("Error: no path found. Either no path exists or the start or end positions are on occupied spaces")
			self.grid_path = []
			return

		# get movements from list of points and travel along path
		if self.grid_path:
			self.grid_movements = get_grid_movements(self.grid_path)

			if self.grid_movements:
				
				if self._fsm != FSM.GENERATE_POINTS:
					# get out of function if not in correct state
					return
				else:	
					# otherwise move to next state
					self._fsm = FSM.FOLLOW_PATH


	def spin(self):
		rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
		while not rospy.is_shutdown() and self.next_waypoint:
			if self.inflated_map:
			# run only when the map is loaded

				if self._fsm == FSM.GENERATE_POINTS:
					self.grid_movements = None
					self.grid_path = None
					self.generate_path()

				elif self._fsm == FSM.FOLLOW_PATH:
					if self.grid_path and self.grid_movements:
						print(self.grid_path)
						print(self.grid_movements)
						self.move_along_path(self.grid_movements)

				elif self._fsm == FSM.UNSTICK_WALL:
					self.unstick_wall()

				elif self._fsm == FSM.STOP:
					self.stop()
							
		rate.sleep()
		

def main():
	"""Main function."""

	# 1st. initialization of node.
	rospy.init_node("waypoint_patrol_2D")

	# Sleep for a few seconds to wait for the registration.
	rospy.sleep(2)

	# Initialization of the class for the mobile motion.
	waypoint_patrol_2d = WaypointPatrol2D()

	# If interrupted, send a stop command before interrupting.
	rospy.on_shutdown(waypoint_patrol_2d.stop)

	# Robot mobile motion.
	try:
		waypoint_patrol_2d.spin()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
	"""Run the main function."""
	main()
