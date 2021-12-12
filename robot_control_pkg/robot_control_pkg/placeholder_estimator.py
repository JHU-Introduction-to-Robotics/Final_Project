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

class PlaceholderEstimator(Node):
	#global goalPose
	#global laserScan
#	global seenLidarBeacons

	goalPose = None
	laserScan = None
	seenLidarBeacons = []
	goalHeading = 0.0
	retHeading = 0.0
	goalHeadingReached = False
	angVelocity = 0.01
	excludeHeadings = []
	targetLidarBeacon = -1
	closestBeaconCalculated = False
	fullScanComplete = False
	targetLidarBeaconFound = False
	distToGoalPos = -1
	distToGoalLidarBeacon = -1
	startPos = -1

	systemState = 0

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

	def __init__(self):

		super().__init__('PlaceholderEstimator')
		self.scan_subscriber = self.create_subscription(
			LaserScan,
			'/en613/scan',
			self.scan_callback,
			qos_profile=qos_profile_sensor_data)

		self.odom_subscriber = self.create_subscription(
			Odometry,
			'/en613/odom',
			self.odom_callback,
			10)

		self.goal_subscriber = self.create_subscription(
			Pose,
			'/en613/goal',
			self.goal_callback,
			10)

		self.publisher_ = self.create_publisher(Pose, '/en613/state_est', 10)
		self.velPublisher = self.create_publisher(Twist, '/en613/cmd_vel', 10)

		self._tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self._tf_buffer,self)

		self._to_frame = 'odom'
		self._from_frame = 'chassis'

	def scan_callback(self, msg):
		self.laserScan = msg

	def odom_callback(self, msg):
		'''
		This callback currently publishes the transfrom message from odom as a state estimate. 
		This is not the way this node should be implemented.
		'''
		try:
			when = rclpy.time.Time()
			trans = self._tf_buffer.lookup_transform(self._to_frame, self._from_frame,
													 when, timeout=Duration(seconds=5.0))
		except tf2_ros.LookupException:
			self.get_logger().info('Transform isn\'t available, waiting...')
			sleep(1)
			return

		position = msg.pose.pose.position
		orientation = msg.pose.pose.orientation
		T_msg = trans.transform.translation
		R_msg = trans.transform.rotation

		self.distToGoalPos = self.getDistanceBetweenPoints(position.x, position.y,
														   self.goalPose.x, self.goalPose.y)

		if self.distToGoalPos < 1:
			self.get_logger().info('Goal position reached, exiting ..')
			self.moveRobot(0.0, 0.0, 0.0)
			sys.exit(-1)

		if self.closestBeaconCalculated == False:
			self.closestBeaconCalculated = True
			minDist = 10000.0

			for i in range(0, len(self.lidarBeaconLocs)):
				lidarBeaconLoc = self.lidarBeaconLocs[i + 1]
				dist = self.getDistanceBetweenPoints(self.goalPose.x, self.goalPose.y,
													 lidarBeaconLoc[0], lidarBeaconLoc[1])
				if dist < minDist:
					minDist = dist
					self.targetLidarBeacon = i + 1

		targetLidarBeaconLoc = self.lidarBeaconLocs[self.targetLidarBeacon]
		self.distToGoalLidarBeacon = self.getDistanceBetweenPoints(position.x, position.y,
																   targetLidarBeaconLoc[0],
																   targetLidarBeaconLoc[1])
		angVel = 0.0
		xVel = 0.0

		self.moveRobot(xVel, 0.0, angVel) # stop robot

		if self.laserScan is not None:

			intensities = self.laserScan.intensities
			ranges = self.laserScan.ranges
			range_min = self.laserScan.range_min
			range_max = self.laserScan.range_max

			curHeading = self.convertQuarternionToEulerAngle(orientation.x, orientation.y,
															 orientation.z, orientation.w)

			#self.get_logger().info('Heading: {0}, Goal Heading: {1}'.format(curHeading, self.goalHeading))
			#self.get_logger().info('Exclude: {0}'.format(self.excludeHeadings))

			if self.systemState == 0:

				if curHeading > (self.goalHeading - 0.009) and curHeading < (self.goalHeading + 0.009):
					self.excludeHeadings.append(self.goalHeading)
					rangeIsGood = True

					for i in range(0, len(ranges)):
						if ranges[i] < 1:
							rangeIsGood = False

					if rangeIsGood == False:
						angVel = 0.01

						while (True):

							self.goalHeading = self.goalHeading + math.pi / 2

							if self.goalHeading > math.pi:
								self.goalHeading = math.pi - self.goalHeading

							if self.goalHeading not in self.excludeHeadings:
								break
					else:
						self.excludeHeadings = []
						self.excludeHeadings.append(self.goalHeading)
						self.retHeading = self.goalHeading

						if self.goalHeading == 0:
							self.excludeHeadings.append(math.pi)
							self.goalHeading = math.pi / 2
						elif self.goalHeading == math.pi:
							self.excludeHeadings.append(0)
							self.goalHeading = math.pi / 2
						else:
							self.excludeHeadings.append(-self.goalHeading)
							self.goalHeading = 0

						self.startPos = position
						self.systemState = 1
				else:
					angVel = 0.01

			elif self.systemState == 1:
				rangeIsGood = True

				for i in range(0, len(ranges)):
					if ranges[i] < 0.5:
						rangeIsGood = False

				if rangeIsGood == False:
					xVel = 0.0
					self.systemState = 0
				else:
					xVel = 0.01
					distTraveled = self.getDistanceBetweenPoints(self.startPos.x, self.startPos.y,
																 position.x, position.y)

					if distTraveled >= 1.0 and self.targetLidarBeaconFound == False:
						if self.distToGoalLidarBeacon < 2:
							self.startPos = position
							self.systemState = 2

			elif self.systemState == 2:
				angVel = 0.01
				desiredHeading = self.retHeading + math.pi

				if desiredHeading > math.pi:
					desiredHeading = math.pi - self.retHeading

				if curHeading > (desiredHeading - 0.009) and curHeading < (desiredHeading + 0.009):
					self.fullScanComplete = True

				if curHeading > (self.retHeading - 0.009) and curHeading < (self.retHeading + 0.009):
					if self.fullScanComplete == True:
						self.fullScanComplete = False
						angVel = 0.0
						self.systemState = 1

				for i in range(0, len(intensities)):
					#if intensities[i] > 0:
						#self.get_logger().info(
						#	'Lidar Beacon {0} is at range {1} m'.format(intensities[i], ranges[i]))
					if intensities[i] == self.targetLidarBeacon:
						self.targetLidarBeaconFound = True
						self.get_logger().info(
							'Target Lidar Beacon {0} is at range {1} m'.format(intensities[i], ranges[i]))
						self.fullScanComplete = False
						angVel = 0.0
						self.systemState = 1
						break

		self.moveRobot(xVel, 0.0, angVel)

		T = np.array([T_msg.x,T_msg.y,T_msg.z])
		R = np.array([R_msg.x,R_msg.y,R_msg.z,R_msg.w])

		X1 = np.array([position.x,position.y,position.z])
		Q1 = np.array([orientation.x,orientation.y,orientation.z,orientation.w])

		self.publish_pose(X1,Q1)

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

	def isGoalPositionReached(self, curPose):

		if self.goalPose is not None:

			isXinRange = curPose.x > (self.goalPose.x - 0.5) and curPose.x < (self.goalPose.x + 0.5)
			isYinRange = curPose.y > (self.goalPose.y - 0.5) and curPose.y < (self.goalPose.y + 0.5)

			return (isXinRange and isYinRange)
		else:
			return True

	def getDistanceBetweenPoints(self, x1, y1, x2, y2):
		return math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))

	def convertQuarternionToEulerAngle(self, x, y, z, w):
		siny_cosp = 2 * ((w * z) + (x * y))
		cosy_cosp = 1 - 2 * ((y * y) + (z * z))
		yaw = math.atan2(siny_cosp, cosy_cosp)

		return yaw

	def goal_callback(self, msg):
		self.goalPose = msg.position
		#self.get_logger().info('Goal Pose: {0}'.format(self.goalPose))

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
