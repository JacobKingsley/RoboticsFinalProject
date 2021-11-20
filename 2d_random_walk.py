#!/usr/bin/env python

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: November 15, 2021
# CS 81 Final Project
# Robot Movement Controller 2D Simulation -- Random Walk Method


import math
import random
import numpy as np

import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'

FREQUENCY = 10 #Hz.

LINEAR_VELOCITY = 0.25 # m/s
ANGULAR_VELOCITY = math.pi/2 # rad/s

MIN_THRESHOLD_DISTANCE = 0.6 # m, threshold distance, should be smaller than range_max

MIN_SCAN_ANGLE_FRONT = -35 / 180 * math.pi
MAX_SCAN_ANGLE_FRONT = 35 / 180 * math.pi

# time delay after rotation to prevent new rotation from starting before current one ends
DT = .5


class RandomWalk_2D():
	def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
		front_scan_angle=[MIN_SCAN_ANGLE_FRONT, MAX_SCAN_ANGLE_FRONT]):
		"""Constructor."""

		# Setting up publishers/subscribers.
		# Setting up the publisher to send velocity commands.
		self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
		# Setting up subscriber receiving messages from the laser.
		self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

		# Parameters.
		self.linear_velocity = linear_velocity # Constant linear velocity set.
		self.angular_velocity = angular_velocity # Constant angular velocity set.
		self.min_threshold_distance = min_threshold_distance
		self.front_scan_angle = front_scan_angle
		self.front_scan_angle = front_scan_angle
		
		# Flag used to control the behavior of the robot.
		self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

	def move(self, linear_vel, angular_vel):
		"""Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
		# Setting velocities.
		twist_msg = Twist()

		twist_msg.linear.x = linear_vel
		twist_msg.angular.z = angular_vel
		self._cmd_pub.publish(twist_msg)

	def stop(self):
		"""Stop the robot."""
		twist_msg = Twist()
		self._cmd_pub.publish(twist_msg)

	def _laser_callback(self, msg):
		# checks for a close obstacle in front of the robot if it has not found one yet
		if not self._close_obstacle:
			start_angle_i = int(round((self.front_scan_angle[0] - msg.angle_min) / msg.angle_increment))
			end_angle_i = int(round((self.front_scan_angle[1] - msg.angle_min) / msg.angle_increment))

			ranges_front = msg.ranges[start_angle_i : end_angle_i + 1]

			f_min_value = np.min(ranges_front)

			# if the closest obstacle is within the set threshold distance
			if f_min_value < self.min_threshold_distance:
				self._close_obstacle = True


	def spin(self):
		rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.

		# keeps track whether or not a new rotation to new angle is starting or whether to continue previously calculated rotation
		start_rotation = True
		turn_angle = 0
		turn_duration = 0
		turn_start_time = 0
		
		while not rospy.is_shutdown():
			if not self._close_obstacle:
				self.move(self.linear_velocity,0)
			else:
				self.stop()
				# if at the beginning of a rotation, calculate a random amount to turn
				if start_rotation == True:
					turn_angle = random.uniform(-math.pi, math.pi)
					turn_duration = abs(turn_angle) / self.angular_velocity
					turn_start_time = rospy.get_rostime()
					start_rotation = False

				if turn_angle < 0:
					ang_vel = self.angular_velocity
				else:
					ang_vel = -self.angular_velocity
				
				while rospy.get_rostime() - turn_start_time < rospy.Duration(turn_duration):
					self.move(0, ang_vel)

					rate.sleep()
				rospy.sleep(DT)

			
				self._close_obstacle = False
				start_rotation = True            
			
			rate.sleep()
			

def main():
	"""Main function."""

	# initialization of node.
	rospy.init_node("random_walk_2d")

	# Sleep for a few seconds to wait for the registration.
	rospy.sleep(2)

	random_walk_2d = RandomWalk_2D()

	rospy.on_shutdown(random_walk_2d.stop)

	try:
		random_walk_2d.spin()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
	"""Run the main function."""
	main()
