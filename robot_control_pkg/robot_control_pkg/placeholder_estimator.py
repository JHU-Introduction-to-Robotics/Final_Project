'''
####################
Sahil Sharma
12/12/21
EN605.613 - Introduction to Robotics
Final Project
Maze Navigator
-------
This file contains the estimator/controller/planner for navigating the robot from a random starting position to a random
goal location. The file name was left for convenience/compatibility reasons but the controller/planner/estimator are all
in this file.The general control of the robot is handled by a state machine which is commented/laid out below
==================================
Copyright 2020,
The Johns Hopkins University Applied Physics Laboratory LLC (JHU/APL).
All Rights Reserved.
####################
'''

import math
import sys

import tf2_ros
import rclpy
from time import sleep
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import qos_profile_sensor_data

# class contains the estimator/planner/controller all in one
class PlaceholderEstimator(Node):

	goalPose = None # goal position
	laserScan = None # current laser scan results
	goalHeading = 0.0 # desired goal heading
	retHeading = 0.0 # heading to return to after 360 scan in in the scanning state
	excludeHeadings = [] # headings to exclude when looking for a new one so we do not go backwards
	targetLidarBeacon = -1 # target lidar beacon, which is the beacon closest to the goal position
	closestBeaconCalculated = False # ensures we only calculate closest lidar beacon once
	fullScanComplete = False # during the scan stage this ensure the robot does a full 360 before moving again
	targetLidarBeaconFound = False # set to true when target lidar beacon is found by the periodic 360 degree scans
	distToGoalPos = -1 # current distance between robot and goal position
	distToGoalLidarBeacon = -1 # current distance between robot and target lidar beacon
	startPos = -1 # start position for linear movement

	systemState = 0 # controller/estimator state for state machine

	# lidar beacon locations in maze
	lidarBeaconLocs = {
						1: [-1.95708, -1.36105],
						2: [-1.95708, -0.12304],
						3: [-1.95708, 1.11498],
						4: [-1.95708, -2.59907],
						5: [-1.95708, -3.83708],
						6: [1.61298, -4.90807],
						7: [2.83795, -4.90807],
						8: [4.06293, -4.90807],
						9: [0.388, -4.90807],
						10: [-0.83697, -4.90807],
						11: [5.21728, -2.1],
						12: [5.21728, -3.9],
						13: [1.68157, 2.19017],
						14: [-0.17, 2.19017],
						15: [-0.402, -1.40765],
						16: [-0.572, -1.40765],
						17: [-0.402, -0.25171],
						18: [-0.572, -0.25171],
						19: [-0.402, -2.5636],
						20: [-0.572, -2.5636],
						21: [0.37432, 0.73833],
						22: [0.37432, 0.90833],
						23: [3.23756, -2.00439],
						24: [3.23756, -2.17561],
						25: [2.4, -3.56615],
						26: [1.89007, -3.73615],
						27: [0.78313, -2.61343],
						28: [1.444, -3.04178],
						29: [1.839, -0.74021],
						30: [4.17214, 0.64628]
					}

	# class constructor
	def __init__(self):

		super().__init__('PlaceholderEstimator')

		# subscribes node to laser scan updates
		self.scan_subscriber = self.create_subscription(
			LaserScan,
			'/en613/scan',
			self.scan_callback,
			qos_profile=qos_profile_sensor_data)

		# subscribes node to odometry updates
		self.odom_subscriber = self.create_subscription(
			Odometry,
			'/en613/odom',
			self.odom_callback,
			10)

		# subscribes node to goal position published by spawner
		self.goal_subscriber = self.create_subscription(
			Pose,
			'/en613/goal',
			self.goal_callback,
			10)

		# makes node pose publisher, but this publishing is not used since the control of the robot is handled in this file
		self.publisher_ = self.create_publisher(Pose, '/en613/state_est', 10)

		# makes node velocity publisher to move robot based on controller calculations
		self.velPublisher = self.create_publisher(Twist, '/en613/cmd_vel', 10)

		self._tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self._tf_buffer,self)

		self._to_frame = 'odom'
		self._from_frame = 'chassis'

	# stores of laser scan results for use in odometry callback
	# @param msg is laser scan message to store off
	def scan_callback(self, msg):
		self.laserScan = msg

	# Acts as the controller/estimator/planner for robot
	# @param msg is odometry message containing the robots current position and orientation
	def odom_callback(self, msg):

		# catches the exception if the transform is not available yet
		try:
			when = rclpy.time.Time()
			trans = self._tf_buffer.lookup_transform(self._to_frame, self._from_frame,
													 when, timeout=Duration(seconds=5.0))
		except tf2_ros.LookupException:
			self.get_logger().info('Transform isn\'t available, waiting...')
			sleep(1)
			return

		# stores of the robots position and orientation as well as translation/rotation for use below
		position = msg.pose.pose.position
		orientation = msg.pose.pose.orientation
		T_msg = trans.transform.translation
		R_msg = trans.transform.rotation

		# calculates current distance between robot and goal position
		self.distToGoalPos = self.getDistanceBetweenPoints(position.x, position.y,
														   self.goalPose.x, self.goalPose.y)

		# stops running controller when robot is within 1 m of goal position
		if self.distToGoalPos < 1:
			self.get_logger().info('Goal position reached, exiting ..')
			self.moveRobot(0.0, 0.0, 0.0)
			sys.exit(-1)

		# calculates the closest lidar beacon to the goal position once and never again
		if self.closestBeaconCalculated == False:

			self.closestBeaconCalculated = True # ensures we do not keep repeating this calculation
			minDist = 10000.0

			# finds the lidar beacon closest to the goal positon and saves it off
			for i in range(0, len(self.lidarBeaconLocs)):
				lidarBeaconLoc = self.lidarBeaconLocs[i + 1]
				dist = self.getDistanceBetweenPoints(self.goalPose.x, self.goalPose.y,
													 lidarBeaconLoc[0], lidarBeaconLoc[1])

				# keeps updating closest beacon until we have looked at them all
				if dist < minDist:
					minDist = dist
					self.targetLidarBeacon = i + 1

		# calculates current distance between robot and target lidar beacon
		targetLidarBeaconLoc = self.lidarBeaconLocs[self.targetLidarBeacon]
		self.distToGoalLidarBeacon = self.getDistanceBetweenPoints(position.x, position.y,
																   targetLidarBeaconLoc[0],
																   targetLidarBeaconLoc[1])

		# sets desires angular and linear velocity to 0 for now
		angVel = 0.0
		xVel = 0.0

		self.moveRobot(xVel, 0.0, angVel) # stop robot until next set of calculations are done by controller

		# ensures laser scan is valid
		if self.laserScan is not None:

			# saves of relevant scan variables for use below
			intensities = self.laserScan.intensities
			ranges = self.laserScan.ranges
			range_min = self.laserScan.range_min
			range_max = self.laserScan.range_max

			# calculates current robot heading in radians from the quaternion
			curHeading = self.convertQuarternionToEulerAngle(orientation.x, orientation.y,
															 orientation.z, orientation.w)

			#self.get_logger().info('Heading: {0}, Goal Heading: {1}'.format(curHeading, self.goalHeading))
			#self.get_logger().info('Exclude: {0}'.format(self.excludeHeadings))

			# Initial state for robot where it attempts to find its next heading either (0, 90, 180, 270, or 360 degrees)
			if self.systemState == 0:

				# if we have reached the goal heading
				if curHeading > (self.goalHeading - 0.009) and curHeading < (self.goalHeading + 0.009):

					self.excludeHeadings.append(self.goalHeading) # ensure we skip this on the next pass
					rangeIsGood = True # checks if the range of the closest wall is far enough

					# determines if distances to wall are acceptable from all the scan results
					for i in range(0, len(ranges)):

						# ensure closest wall is atleast a meter away to allow linear travel
						if ranges[i] < 1:
							rangeIsGood = False

					# if range to wall is not good keep rotating after updatng goal heading
					if rangeIsGood == False:
						angVel = 0.01 # sets angular velocty for rotation

						# updates goal heading to something we have not tried before
						while (True):

							self.goalHeading = self.goalHeading + math.pi / 2

							# heading goes negative so this account for that
							if self.goalHeading > math.pi:
								self.goalHeading = math.pi - self.goalHeading

							# leaves loop when new unique heading is found
							if self.goalHeading not in self.excludeHeadings:
								break

					# if range to wall is good then we have found our new heading
					else:

						# updates headings to exclude so we do not go backward the next time we are in this state
						self.excludeHeadings = []
						self.excludeHeadings.append(self.goalHeading)
						self.retHeading = self.goalHeading

						# accounts of the negative headings
						if self.goalHeading == 0:
							self.excludeHeadings.append(math.pi)
							self.goalHeading = math.pi / 2
						elif self.goalHeading == math.pi:
							self.excludeHeadings.append(0)
							self.goalHeading = math.pi / 2
						else:
							self.excludeHeadings.append(-self.goalHeading)
							self.goalHeading = 0

						# saves of starting position and moves state to the linear movement state
						self.startPos = position
						self.systemState = 1

				# if we have not reached our goal heading keep rotating
				else:
					angVel = 0.01

			# this is the linar movement state that happens after the heading is determined above
			elif self.systemState == 1:
				rangeIsGood = True # ensure we keep a reasonable distance from the wall

				# checks range to wall is acceptable based on laser scan
				for i in range(0, len(ranges)):
					if ranges[i] < 0.5:
						rangeIsGood = False

				# if the wall is too close its time to find a new heading
				if rangeIsGood == False:
					xVel = 0.0
					self.systemState = 0 # transition back to heading state

				# if wall is still far enough away keep moving towards it
				else:
					xVel = 0.01

					# calculates distance traveled from the point where we last found our goal heading
					distTraveled = self.getDistanceBetweenPoints(self.startPos.x, self.startPos.y,
																 position.x, position.y)

					# as we come within range of the target lidar beacon we begin 360 lidar scans to find it
					if distTraveled >= 1.0 and self.targetLidarBeaconFound == False:

						# begin 360 lidar scans once we are within 2 m of it
						if self.distToGoalLidarBeacon < 2:
							self.startPos = position
							self.systemState = 2

			# this is the 360 lidar scan state used to find the target lidar beacon
			elif self.systemState == 2:
				angVel = 0.01
				desiredHeading = self.retHeading + math.pi # this is the heading we want to return to once scan is complete

				# adjusts for negative angles
				if desiredHeading > math.pi:
					desiredHeading = math.pi - self.retHeading

				# ensure we complete the full 360 degree scan
				if curHeading > (desiredHeading - 0.009) and curHeading < (desiredHeading + 0.009):
					self.fullScanComplete = True

				# once the full 360 scan is complete and we return to the original heading we jump back to the linear
				# driving state above
				if curHeading > (self.retHeading - 0.009) and curHeading < (self.retHeading + 0.009):
					if self.fullScanComplete == True:
						self.fullScanComplete = False
						angVel = 0.0
						self.systemState = 1

				# determines if we see the target lidar beacon during our 360 degree scan
				for i in range(0, len(intensities)):

					# if we have found the target lidar beacon in our sweep we return to the linear drive state
					# and begin driving towards it with the idea being that we will end up within 1 meter of our goal
					# position
					if intensities[i] == self.targetLidarBeacon:
						self.targetLidarBeaconFound = True
						self.get_logger().info(
							'Target Lidar Beacon {0} is at range {1} m'.format(intensities[i], ranges[i]))
						self.fullScanComplete = False
						angVel = 0.0
						self.systemState = 1
						break

		self.moveRobot(xVel, 0.0, angVel) # moves robot based on state machine calculations

		# this code below is legacy software and not used
		T = np.array([T_msg.x,T_msg.y,T_msg.z])
		R = np.array([R_msg.x,R_msg.y,R_msg.z,R_msg.w])

		X1 = np.array([position.x,position.y,position.z])
		Q1 = np.array([orientation.x,orientation.y,orientation.z,orientation.w])

		self.publish_pose(X1,Q1)

	# moves robot based on specified parameters
	# @param xVel is desired linear x velocity in m/s
	# @param yVel is desired linear x velocity in m/s
	# @param angVel is desired angular velocity in rad/s
	def moveRobot(self, xVel, yVel, angVel):
		velMsg = Twist()

		velMsg.linear.x = xVel
		velMsg.linear.y = yVel
		velMsg.linear.z = 0.0
		velMsg.angular.x = 0.0
		velMsg.angular.y = 0.0
		velMsg.angular.z = angVel

		'''
		if xVel > 0:
			self.get_logger().info('x Velocity {0} ,m/s'.format(velMsg.angular.x))
		if yVel > 0:
			self.get_logger().info('y Velocity {0} ,m/s'.format(velMsg.angular.y))
		if angVel > 0:
			self.get_logger().info('Angular Velocity {0} rad/s'.format(velMsg.angular.z))
		'''
		self.velPublisher.publish(velMsg)

	# calculates linear distance between 2 points (x1, y1) and (x2, y2)
	# @param x1 is x coordinate of point 1
	# @param y1 is y coordinate of point 1
	# @param x2 is x coordinate of point 2
	# @param y2 is y coordinate of point 2
	def getDistanceBetweenPoints(self, x1, y1, x2, y2):
		return math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))

	# converts quaternion to standard yaw heading in radians
	def convertQuarternionToEulerAngle(self, x, y, z, w):
		siny_cosp = 2 * ((w * z) + (x * y))
		cosy_cosp = 1 - 2 * ((y * y) + (z * z))
		yaw = math.atan2(siny_cosp, cosy_cosp)

		return yaw

	# stores of goal position for use in controller
	def goal_callback(self, msg):
		self.goalPose = msg.position
		#self.get_logger().info('Goal Pose: {0}'.format(self.goalPose))

	# publishes pose but this is legacy software and not used for my implementation
	def publish_pose(self,X1,Q1):

		msg = Pose()

		msg.position.x = X1[0]
		msg.position.y = X1[1]
		msg.position.z = X1[2]
		msg.orientation.x = Q1[0]
		msg.orientation.y = Q1[1]
		msg.orientation.z = Q1[2]
		msg.orientation.w = Q1[3]

		self.publisher_.publish(msg)

# runs estimator/controller/planner node
def main(args=None):
    rclpy.init(args=args)

    estimator = PlaceholderEstimator()

    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
