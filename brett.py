#!/usr/bin/env python
# Author: Brett Kidman 2021
# import of relevant libraries.
import rospy # module for ROS APIs
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

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
		# Setting up the subscribers.
		self.image_sub = rospy.Subscriber("/image", Image, _image_callback, queue_size=10)
		self.br = CvBridge()
		self.frequency = 10

		# Sleep important for allowing time for registration to the ROS master.
		rospy.sleep(2)

	def _image_callback(self, msg):
		# get uncompressed image from camera
		image = self.br.imgmsg_to_cv2(msg)
		print(msg)
		print(image)

	def spin(self):
		rate = rospy.Rate(self.frequency)
		while not rospy.is_shutdown():
			rate.sleep()

	

def main():
	"""main function."""
	rospy.init_node("robot_cam")
	cam = Robot_Camera()

if __name__ == "__main__":
	""" run the main function. """
	main()