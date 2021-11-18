#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Brett M Kidman    
# Date: 9/21/2021

# Import of python modules.
import math # use of pi.
import random
import numpy as np

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.3 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
MIN_THRESHOLD_DISTANCE = 0.6 # m, threshold distance, should be smaller than range_max

# .5, -10+10

# Field of view in radians that is checked in front of the robot (feel free to tune)
#MIN_SCAN_ANGLE_RAD = 2*math.pi-(20.0 / 180 * math.pi;
#MAX_SCAN_ANGLE_RAD = 20.0 / 180 * math.pi;
MIN_SCAN_ANGLE_RAD = 0
MAX_SCAN_ANGLE_RAD = 1/6*math.pi

LASER_FRONT = math.pi
MIN_SCAN_ANGLE_FRONT = 325 / 180 * math.pi
MAX_SCAN_ANGLE_FRONT = 35 / 180 * math.pi
HALF_FRONT_ANGLE = 35 / 180 * math.pi


class RandomWalk():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD], front_scan_angle=[MIN_SCAN_ANGLE_FRONT, MAX_SCAN_ANGLE_FRONT]):
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
        self.scan_angle = scan_angle
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
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.
        
        
        if not self._close_obstacle:
            # Find the minimum range value between min_scan_angle and
            # max_scan_angle
            # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
            # Note: You have to find the min index and max index.
            # Please double check the LaserScan message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
            ####### TODO: ANSWER CODE BEGIN #######
            # using angle_increment and start angle from laser scan message, find the min and max index of range within scan angle
            #print(msg.ranges)
            #min_index = int((-msg.angle_min+self.scan_angle[0])/msg.angle_increment)
            
            #max_index = int((-msg.angle_min+self.scan_angle[1])/msg.angle_increment)
            #scans_in_range = msg.ranges[min_index:max_index+1]
            #scans_in_range = msg.ranges

            forward_i = int(round((LASER_FRONT - msg.angle_min) / msg.angle_increment))
            forward_i_2 = int(round(((LASER_FRONT + 2 * math.pi) - msg.angle_min) / msg.angle_increment))
            self.forward_scan_distance = msg.ranges[forward_i]

            # print(len(msg.ranges))
            # print(msg.angle_min)
            # print(msg.angle_max)
            # print(msg.angle_increment)


            # check front and set a flag if the robot gets too close to an obstacle
            #f_start_index = int(round((self.front_scan_angle[0] - msg.angle_min) / msg.angle_increment))
            #f_end_index = int(round((self.front_scan_angle[1] - msg.angle_min) / msg.angle_increment))

            #ranges_spliced = msg.ranges[f_start_index : forward_i_2] + msg.ranges[forward_i : f_end_index + 1]

            ranges_front = msg.ranges[0:20] + msg.ranges[340:360]

            #f_min_value = np.min(ranges_spliced)
            f_min_value = np.min(ranges_front)


            #if min(scans_in_range) < self.min_threshold_distance:
            if f_min_value < self.min_threshold_distance:
                self._close_obstacle = True

            ####### ANSWER CODE END #######

    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.

        # keeps track whether or not a new rotation to new angle is starting or whether to continue previously calculated rotation
        start_rotation = True
        turn_angle = 0
        turn_duration = 0
        turn_start_time = 0
        
        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C
            
            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random amount of time
            # after which the flag is set again to False.
            # Use the function move already implemented, passing the default velocities saved in the corresponding class members.

            ####### TODO: ANSWER CODE BEGIN #######
            # if no obstacle, move forward
            if not self._close_obstacle:
                self.move(self.linear_velocity,0)
            # else, turn in place
            else:
                self.stop()
                #print("reached")
                # if at the beginning of a rotation, calculate a random amount to turn
                if start_rotation == True:
                    print("new rotation")
                    turn_angle = random.uniform(-math.pi, math.pi)
                    turn_duration = abs(turn_angle) / self.angular_velocity
                    turn_start_time = rospy.get_rostime()
                    start_rotation = False

                if turn_angle < 0:
                    self.move(0, self.angular_velocity)
                else:
                    self.move(0, -self.angular_velocity)

                if rospy.get_rostime() - turn_start_time >= rospy.Duration(turn_duration):
                    self._close_obstacle = False
                    start_rotation = True            
            
            ####### ANSWER CODE END #######

            rate.sleep()
        

def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("random_walk")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    random_walk = RandomWalk()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(random_walk.stop)

    # Robot random walks.
    try:
        random_walk.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
