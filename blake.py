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

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan


# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'


# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

DT_MODE = 15

# Velocities that will be used
LINEAR_VELOCITY = .8 # m/s
ANGULAR_VELOCITY = math.pi/3 # rad/s

# Target distance from right wall
TARGET_DISTANCE = .5 # m

# constant holding the orientation of the front of the robot in the laser frame
LASER_FRONT = 0
# LASER_FRONT = -math.pi

# constant holding the orientation of the right of the robot in the laser frame
LASER_RIGHT = (LASER_FRONT - math.pi/2)
# LASER_RIGHT = ((((LASER_FRONT - math.pi/2) + math.pi) % 2*math.pi) - math.pi)

LASER_LEFT = LASER_FRONT + math.pi / 2
# LASER_LEFT = ((((LASER_FRONT + math.pi/2) + math.pi) % 2*math.pi) - math.pi)

# Field of view in radians that is checked to the right of the robot for distance to the wall
MIN_SCAN_ANGLE_RIGHT = -180 / 180 * math.pi
MAX_SCAN_ANGLE_RIGHT = 0 / 180 * math.pi

MIN_SCAN_ANGLE_LEFT = 0 / 180 * math.pi
MAX_SCAN_ANGLE_LEFT = 180 / 180 * math.pi

# Field of view in radians that is checked in front of the robot for obstacles
HALF_FRONT_ANGLE = 45.0 / 180 * math.pi
MIN_SCAN_ANGLE_FRONT = LASER_FRONT - HALF_FRONT_ANGLE
MAX_SCAN_ANGLE_FRONT = LASER_FRONT + HALF_FRONT_ANGLE

# MIN_SCAN_ANGLE_FRONT = LASER_FRONT - HALF_FRONT_ANGLE
# MAX_SCAN_ANGLE_FRONT = LASER_FRONT + HALF_FRONT_ANGLE

# minimum distance for registering obstacles in front of the robot
MIN_THRESHOLD_DISTANCE = .7 # m

# PD Controller gain values
KP = 4
KD = 8

# fsm controller for robot's state
class FSM(Enum):
	RANDOM_WALK = 0
	PD_START = 1
	PD_FOLLOW_WALL = 2
	PD_TURN_LEFT = 3
	PD_TURN_RIGHT = 4
	STOP = 5

	RIGHT_WALL = 10
	LEFT_WALL = 11


class Intruder_Patrol_Movement():
	def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, target_distance=TARGET_DISTANCE,
		right_scan_angle=[MIN_SCAN_ANGLE_RIGHT, MAX_SCAN_ANGLE_RIGHT], left_scan_angle=[MIN_SCAN_ANGLE_LEFT, MAX_SCAN_ANGLE_LEFT], front_scan_angle=[MIN_SCAN_ANGLE_FRONT, MAX_SCAN_ANGLE_FRONT], kp=KP, kd=KD, min_threshold_distance=MIN_THRESHOLD_DISTANCE):
		"""Constructor."""

		# Setting up publishers/subscribers.
		# Setting up the publisher to send velocity commands.
		self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
		# Setting up subscriber receiving messages from the laser.
		self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)


		# Parameters.
		self.linear_velocity = linear_velocity # Constant linear velocity set.
		self.angular_velocity = angular_velocity # Angular velocity based on LIDAR scan, but starts at 0

		self.target_distance = target_distance
		self.right_scan_angle = right_scan_angle
		self.left_scan_angle = left_scan_angle
		self.front_scan_angle = front_scan_angle
		self.min_threshold_distance = min_threshold_distance

		# set initial error
		self.err = 0

		# set dt to be the time between steps based on frequency
		self._dt = 1 / FREQUENCY

		self._p = kp
		self._d = kd
		self._errors = [] # list of errors

		self.front_obs = False

		# self._fsm = FSM.RANDOM_WALK
		self._fsm = FSM.PD_START

		self._wall_fsm = FSM.RIGHT_WALL

	
	def move(self, linear_vel, angular_vel):
		"""Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
		# Setting velocities.
		twist_msg = Twist()

		twist_msg.linear.x = linear_vel
		twist_msg.angular.z = angular_vel
		self._cmd_pub.publish(twist_msg)

	def stop(self):
		print("stopping")
		"""Stop the robot."""
		twist_msg = Twist()
		self._cmd_pub.publish(twist_msg)

	def _laser_callback(self, msg):
		"""Processing of laser message."""
		# NOTE: assumption: the one at angle 0 corresponds to the front.
		
		# Find the minimum range value between min_scan_angle and max_scan_angle
		# to find distance to wall for the PD-Controller

		# find indices that hold the ranges within the field of view
		forward_i = int(round((LASER_FRONT - msg.angle_min) / msg.angle_increment))
		self.forward_scan_distance = msg.ranges[forward_i]
		
		right_start_index = int(round((self.right_scan_angle[0] - msg.angle_min) / msg.angle_increment))
		right_end_index = int(round((self.right_scan_angle[1] - msg.angle_min) / msg.angle_increment))

		left_start_index = int(round((self.left_scan_angle[0] - msg.angle_min) / msg.angle_increment))
		left_end_index = int(round((self.left_scan_angle[1] - msg.angle_min) / msg.angle_increment))

		# find the minimum distance value in this range
		min_scan_r = min(msg.ranges[right_start_index:right_end_index])
		min_scan_l = min(msg.ranges[left_start_index:left_end_index])

		if min_scan_r <= min_scan_l:
			self._wall_fsm = FSM.RIGHT_WALL
		else:
			self._wall_fsm = FSM.LEFT_WALL

		# set wall-follow error as this minimum distance to the right of the robot
		self.err = self.target_distance - min_scan_r


		# check front and set a flag if the robot gets too close to an obstacle
		f_start_index = int(round((self.front_scan_angle[0] - msg.angle_min) / msg.angle_increment))
		f_end_index = int(round((self.front_scan_angle[1] - msg.angle_min) / msg.angle_increment))

		f_min_value = np.min(msg.ranges[f_start_index:f_end_index+1])
		f_min_index = msg.ranges[f_start_index:f_end_index+1].index(f_min_value)
		f_min_angle = (((msg.angle_min + (f_min_index * msg.angle_increment)) + math.pi ) % 2*math.pi ) - math.pi
		
		if f_min_value < self.min_threshold_distance:
			self.front_obs = True

		# determine fsm state by the front_obs flag and start flag
		if self._fsm != FSM.RANDOM_WALK and self._fsm != FSM.STOP:
			print("PD WALL")
			if self.front_obs:
				self.pd_reset()

				print("PD ANGLE: " + str(f_min_angle))
				if f_min_angle >= LASER_FRONT:
					self._fsm = FSM.PD_TURN_LEFT
				else:
				# 	self._fsm = FSM.PD_TURN_RIGHT
					self._fsm = FSM.PD_TURN_LEFT

			elif self._fsm != FSM.PD_START:
				self._fsm = FSM.PD_FOLLOW_WALL

	def pd_step(self, err, dt):
		# Steps the PD forward with given error and dt interval. Returns the calculated actuation as the output
		
		# add the error to the list of errors
		self._errors.append(err)

		# P controller
		u = self._p * err
		d = 0

		# add in the D controller
		if(len(self._errors) > 1):
			d = (self._errors[-1] - self._errors[-2]) / dt
			u +=  self._d * d

		# print statement for testing
		# print("e: {} \t d: {} \t u: {}".format(err, d, u))
			
		return u

	def pd_reset(self):
		# reset the PD Controller error values when it has to turn and start the wall following over
		self._errors = []


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

						
			"""using only timer"""
			# Get current time.
			start_time = rospy.get_rostime()
			# publish rotation until enough time elapses so the robot turns the given angle
			while rospy.get_rostime() - start_time < rospy.Duration(abs(angle) / self.angular_velocity):
				self.move(0, ang_vel)
			
				rate.sleep()

	def drive(self, distance):
		rate = rospy.Rate(FREQUENCY)

		if not rospy.is_shutdown():

			# if distance is negative, move backwards
			if distance < 0:
				lin_vel = - self.linear_velocity
			else: # if distance is positive, move forward
				lin_vel = self.linear_velocity
			
			"""using only timer"""
			# Get current time.
			start_time = rospy.get_rostime()
			# publish motion until enough time elapses so the robot moves the given distance
			while rospy.get_rostime() - start_time < rospy.Duration(abs(distance) / self.linear_velocity):
				self.move(lin_vel, 0)


				rate.sleep()
				

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


	def follow_wall(self):
		# move at constant linear velocity and angular velocity based on error

		# calculate actuation with self.step() method
		ang_vel = self.pd_step(self.err, self._dt)
		self.move(self.linear_velocity, ang_vel)

	def random_walk(self):
		rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
		if not rospy.is_shutdown() and self._fsm == FSM.RANDOM_WALK:
			# Keep looping until user presses Ctrl+C
			
			# If the flag self._close_obstacle is False, the robot should move forward.
			# Otherwise, the robot should rotate for a random amount of time
			# after which the flag is set again to False.
			# Use the function move already implemented, passing the default velocities saved in the corresponding class members.

			if not self.front_obs:
				self.move(LINEAR_VELOCITY, 0)

			else:
				self.random_rotate()
				self.front_obs = False
				
			rate.sleep()

	def random_rotate(self):
		rand_angle = 0
		while abs(rand_angle) <= HALF_FRONT_ANGLE:
			rand_angle = 2 * math.pi * random.random() - math.pi
			print("att " + str(rand_angle))
		
		print(rand_angle)

		self.rotate(rand_angle)



	def spin(self):
		rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.

		# keep track of time to switch between movement methods
		self.start_time = rospy.get_rostime()

		while not rospy.is_shutdown():

			self.curr_time = rospy.get_rostime()

			# if self.curr_time - self.start_time >= rospy.Duration(DT_MODE):
			# 	if self._fsm == FSM.RANDOM_WALK:
			# 		self._fsm = FSM.PD_START
			# 		print("PD")
			# 	else:
			# 		self._fsm = FSM.RANDOM_WALK
			# 		print("RW")

			# 	self.start_time = rospy.get_rostime()
				

			# follow right wall until robot finds an obstacle
			if self._fsm == FSM.PD_FOLLOW_WALL:
				self.follow_wall()

			# turn left until not blocked by obstacle
			elif self._fsm == FSM.PD_TURN_LEFT:
				print("lll")
				self.move(0, self.angular_velocity)
			elif self._fsm == FSM.PD_TURN_RIGHT:
				print("rrr")
				self.move(0, - self.angular_velocity)

			# go forward at the beginning of the script until the robot finds a wall
			elif self._fsm ==FSM.PD_START:
				self.move(self.linear_velocity, 0)

			elif self._fsm == FSM.RANDOM_WALK:
				self.random_walk()

			elif self._fsm == FSM.STOP:
				self.stop()

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
