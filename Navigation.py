#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Import of python modules.
import math # use of pi.
import random
import numpy as np

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan # message type for scan

""" ************* CONSTANTS ******************* """
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # change from base_scan to scan for turtlebot
DEFAULT_STRING_TOPIC = '/strings'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used
LINEAR_VELOCITY = 0.3 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance
MIN_THRESHOLD_DISTANCE = 0.6 # meters

# Scan angle of robot
SCAN_ANGLE = 20 # degrees, total FOV multiplied by 2 (to left and right of robot)


""" *********** RANDOM WALK CLASS ********** """
class RandomWalk():
	def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
		scan_angle=SCAN_ANGLE, frequency = FREQUENCY):
		"""Constructor."""

		# Setting up publishers/subscribers.
		# Setting up the publisher to send velocity commands.
		self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
		# Setting up subscriber receiving messages from the laser.
		self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
		self._string_sub = rospy.Subscriber(DEFAULT_STRING_TOPIC, String, self._string_callback, queue_size=2)

		# Parameters.
		self.linear_velocity = linear_velocity # Constant linear velocity set.
		self.angular_velocity = angular_velocity # Constant angular velocity set.
		self.min_threshold_distance = min_threshold_distance
		self.scan_angle = scan_angle
		self.frequency = frequency
		
		self.is_close = False # designates if close to wall / obstacle
		self.persons_found = False # keeps track of whether or not all target personnel have been identified

		rospy.sleep(2)


	# moves the robot with inputted linear and angular velocity
	def move(self, linear_vel, angular_vel):
		# Setting velocities.
		twist_msg = Twist()

		twist_msg.linear.x = linear_vel
		twist_msg.angular.z = angular_vel
		self._cmd_pub.publish(twist_msg)

	
	# sends velocities to 0 to stop robot
	def stop(self):
		twist_msg = Twist()
		self._cmd_pub.publish(twist_msg)

	# process laser data
	def _laser_callback(self, msg):
		if not self.is_close: # check if obstacle within scan range

			# for turtlebot, index 0 starts in front. Angle increment is 1 degree. Need to check to left and right which is start and end of ranges list.  
			ranges_front = msg.ranges[0:self.scan_angle+1] + msg.ranges[360-self.scan_angle:361]
			f_min_value = np.min(ranges_front)

			if f_min_value < self.min_threshold_distance:
				self.is_close = True

	def _string_callback(self, msg):
		print(msg)
		if msg.data == "PERSONS FOUND":
			self.persons_found = True

	# random walk cycle. If no wall or obstacle in front, keep walking forward. Else, rotate. 
	def spin(self):
		rate = rospy.Rate(self.frequency)
		# keeps track whether or not a new rotation to new angle is starting or whether to continue previously calculated rotation
		start_rotation = True
		turn_angle = 0
		turn_duration = 0
		turn_start_time = 0
		
		while not rospy.is_shutdown() and not self.persons_found:
			# if no obstacle, move forward
			if not self.is_close:
				self.move(self.linear_velocity,0)
			# else, turn in place
			else:
				self.stop()
				# if at the beginning of a rotation, calculate a random amount to turn
				if start_rotation == True:
					turn_angle = random.uniform(-math.pi, math.pi)
					turn_duration = abs(turn_angle) / self.angular_velocity
					turn_start_time = rospy.get_rostime()
					start_rotation = False

				if turn_angle < 0:
					self.move(0, self.angular_velocity)
				else:
					self.move(0, -self.angular_velocity)

				if rospy.get_rostime() - turn_start_time >= rospy.Duration(turn_duration):
					self.is_close = False
					start_rotation = True

			rate.sleep()

		if self.persons_found:
			print("All personnel found. Stopping navigation.")
		
""" ************* MAIN CODE *************** """
def main():
	rospy.init_node("random_walk")

	# Initialization of the class for the random walk.
	random_walk = RandomWalk()

	# If interrupted, send a stop command before interrupting.
	rospy.on_shutdown(random_walk.stop)

	# initiate random walk
	try:
		random_walk.spin()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
	"""Run the main function."""
	main()
