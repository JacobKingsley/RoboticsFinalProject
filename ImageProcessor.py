#!/usr/bin/env python
# Author: Brett Kidman 2021

# Author: Blake Danziger, Burke Jaeger, Brett Kidman, Jacob Kingsley
# Date: Fall 2021
# CS 81/281 Final Project
# Robot Image Processor

# import of relevant libraries.
import rospy # module for ROS APIs
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import cv2

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
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self._image_callback, queue_size=10)
		self.string_pub = rospy.Publisher('/strings', String, queue_size=10) # used to let navigator node know when all personnel have been found
		self.br = CvBridge()
		self.frequency = 10

		# make sure receiving an image from image_sub
		self.has_image = False

		self.blue_found = False
		self.red_found = False
		self.green_found = False
		self.yellow_found = False



		# Sleep important for allowing time for registration to the ROS master.
		rospy.sleep(2)

	def _image_callback(self, msg):
		# get uncompressed image from camera
		image = self.br.imgmsg_to_cv2(msg)
		image_fixed = self.br.imgmsg_to_cv2(msg, "bgr8") # process one in bgr8 so can write output image in correct color space
		self.image = image
		self.image_fixed = image_fixed
		self.has_image = True

	def personnel_found(self):
		# publish message denoting that all personnel have been found
		self.string_pub.publish("PERSONS FOUND")

	def identify_entity(self):
		if self.has_image:
			output_image = self.image_fixed

			image_frame = self.image
			blurred_img = cv2.blur(image_frame, (15,15)) # blur out image to reduce small noise
			hsv_frame = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV) # convert to HSV color space

			kernal = np.ones((5,5), "uint8") # image convulution with kernal, used later to reduce noise

			""" STEP 1: IDENTIFY RED """
			# note: color space reversed in processed image so blue = red
			# define lower and upper ranges within HSV color space
			blue_lower = np.array([94, 80, 2], np.uint8)
			blue_upper = np.array([120, 255, 255], np.uint8)
			mask_blue = cv2.inRange(hsv_frame, blue_lower, blue_upper)

			mask_blue = cv2.dilate(mask_blue, kernal)
			ret_b, thresh_b = cv2.threshold(mask_blue, 150, 255, cv2.THRESH_BINARY)

			# generate contours based on all colors within defined HSV range
			_, blue_contours, hierarchy = cv2.findContours(image=thresh_b, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE) 
			blue_max = 0.0

			# find the largest contour for given color. Largest contours of each color will be compared.
			for contour in blue_contours:
				area = cv2.contourArea(contour)
				if area > blue_max:
					blue_max = area
				x, y, w, h = cv2.boundingRect(contour)
				image_frame = cv2.rectangle(output_image, (x,y), (x+w,y+h), (0, 0, 255), 2)
				cv2.putText(output_image, "RED", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255)) # mark the blob in output image for visualization purposes


			""" STEP 2: IDENTIFY BLUE """
			# define lower and upper ranges within HSV color space
			red_lower = np.array([136, 87, 80], np.uint8)
			red_upper = np.array([180, 255, 255], np.uint8)
			mask_red = cv2.inRange(hsv_frame, red_lower, red_upper)

			mask_red = cv2.dilate(mask_red, kernal)
			ret_r, thresh_r = cv2.threshold(mask_red, 150, 255, cv2.THRESH_BINARY)

			# generate contours based on all colors within defined HSV range
			_, red_contours, hierarchy = cv2.findContours(image=thresh_r, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
			red_max = 0.0

			# find the largest contour for given color. Largest contours of each color will be compared.
			for contour in red_contours:
				area = cv2.contourArea(contour)
				if area > red_max:
					red_max = area
				x, y, w, h = cv2.boundingRect(contour)
				image_frame = cv2.rectangle(output_image, (x,y), (x+w,y+h), (255, 0, 0), 2)
				cv2.putText(output_image, "BLUE", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))  # mark the blob in output image for visualization purposes

			""" STEP 3: IDENTIFY GREEN """
			# define lower and upper ranges within HSV color space
			green_lower = np.array([20, 52, 72],np.uint8)
			green_upper = np.array([102, 255, 255],np.uint8)
			mask_green = cv2.inRange(hsv_frame, green_lower, green_upper)

			mask_green = cv2.dilate(mask_green, kernal)

			# generate contours based on all colors within defined HSV range
			_, green_contours, hierarchy = cv2.findContours(image=mask_green, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
			green_max = 0.0

			cv2.imwrite("green_thresh.jpg", mask_green)

			# find the largest contour for given color. Largest contours of each color will be compared.
			for contour in green_contours:
				area = cv2.contourArea(contour)
				if area > green_max:
					green_max = area
				x, y, w, h = cv2.boundingRect(contour)
				image_frame = cv2.rectangle(output_image, (x,y), (x+w,y+h), (0, 255, 0), 2)
				cv2.putText(output_image, "GREEN", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0)) # mark the blob in output image for visualization purposes

			""" STEP 4: IDENTIFY YELLOW """
			"""
			#yellow_lower = np.array([30, 150, 80],np.uint8)
			#yellow_upper = np.array([60, 255, 200],np.uint8)
			yellow_lower = np.array([20, 50, 50],np.uint8)
			yellow_upper = np.array([150, 255, 255],np.uint8)
			mask_yellow = cv2.inRange(hsv_frame, yellow_lower, yellow_upper)
			
			mask_yellow = cv2.dilate(mask_yellow, kernal)

			_, yellow_contours, hierarchy = cv2.findContours(image=mask_yellow, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
			yellow_max = 0.0


			cv2.imwrite("yellow_thresh.jpg", mask_yellow)

			for contour in yellow_contours:
				area = cv2.contourArea(contour)
				if area > yellow_max:
					yellow_max = area
				x, y, w, h = cv2.boundingRect(contour)
				image_frame = cv2.rectangle(output_image, (x,y), (x+w,y+h), (0, 242, 252), 2)
				cv2.putText(output_image, "YELLOW", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 242, 252))

			"""
			yellow_max = 0


			""" STEP 5: DETERMINE IF ONE OF COLORED PERSONNEL IS PRESENT """

			# first make sure atleast one contour above minimum size threshold to filter out noise or if all are 0
			if blue_max < 10 and red_max < 10 and green_max < 10 and yellow_max < 10:
				print("searching...")
				cv2.imwrite("view.jpg", output_image)
				return False

			if red_max > blue_max and red_max > green_max and red_max > yellow_max:
				# BLUE FOUND
				if not self.blue_found:
					self.blue_found = True
					print("BLUE HAS BEEN FOUND")
					cv2.imwrite('BLUE.jpg', output_image)
			elif blue_max > red_max and blue_max > green_max and blue_max > yellow_max:
				# RED FOUND
				if not self.red_found:
					self.red_found = True
					print("RED HAS BEEN FOUND")
					cv2.imwrite('RED.jpg', output_image)

			elif green_max > red_max and green_max > blue_max and green_max > yellow_max:
				# GREEN FOUND
				if not self.green_found:
					self.green_found = True
					print("GREEN HAS BEEN FOUND")
					cv2.imwrite('GREEN.jpg', output_image)
			else:
				# YELLOW FOUND
				if not self.yellow_found:
					self.yellow_found = True
					print("YELLOW HAS BEEN FOUND")
					cv2.imwrite('YELLOW.jpg', output_image)

		return True

	def spin(self):
		# scan every 0.5 seconds until all personnel are identified
		while not self.blue_found or not self.red_found or not self.green_found:
			rospy.sleep(0.5)
			self.identify_entity()

		print("ALL PERSON(S) SUCCESSFULLY IDENTIFIED.")
		self.personnel_found() # lets navigation know task is complete

	

def main():
	"""main function."""
	rospy.init_node("robot_cam")
	cam = Robot_Camera()

	try:
		cam.spin()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
	""" run the main function. """
	main()