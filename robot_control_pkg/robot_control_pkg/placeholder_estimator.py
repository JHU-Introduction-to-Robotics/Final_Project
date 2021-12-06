import math
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

		#if self.isGoalPositionReached(position) == False:

		self.moveRobot(0.0, 0.0, 0.0) # stop robot

		if self.laserScan is not None:
			intensities = self.laserScan.intensities
			ranges = self.laserScan.ranges
			range_min = self.laserScan.range_min
			range_max = self.laserScan.range_max

			beaconSeen = False
			angVel = 0.01

			for i in range(0, len(intensities)):
				if intensities[i] > 0 and intensities[i] not in self.seenLidarBeacons:

					locInScan = float((i + 1) / len(intensities))
					#self.get_logger().info('Loc in scan: {0}'.format(locInScan))

					self.get_logger().info('Lidar Beacon {0} is at range {1} m'.format(intensities[i], ranges[i]))
					beaconSeen = True

					if ranges[i] < 1.5:
						beaconSeen = False
						self.seenLidarBeacons.append(intensities[i])
					#if locInScan > 0.45 and locInScan < 0.55:

					#elif locInScan < 0.15:
					#	beaconSeen = False
					#	angVel = -0.01
					#elif locInScan > 0.85:
					#	beaconSeen = False
					#	angVel = 0.01
				if intensities[i] == 0 and ranges[i] < 0.5:
					beaconSeen = False
					angVel = 0.01

			if beaconSeen == False:
				self.moveRobot(0.0, 0.0, angVel)
			else:
				self.moveRobot(0.01, 0.0, 0.0) # stop robot

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

		if xVel > 0:
			self.get_logger().info('x Velocity {0} ,m/s'.format(velMsg.angular.x))
		if yVel > 0:
			self.get_logger().info('y Velocity {0} ,m/s'.format(velMsg.angular.y))
		if angVel > 0:
			self.get_logger().info('Angular Velocity {0} rad/s'.format(velMsg.angular.z))

		self.velPublisher.publish(velMsg)

	def isGoalPositionReached(self, curPose):

		if self.goalPose is not None:

			isXinRange = curPose.x > (self.goalPose.x - 0.5) and curPose.x < (self.goalPose.x + 0.5)
			isYinRange = curPose.y > (self.goalPose.y - 0.5) and curPose.y < (self.goalPose.y + 0.5)

			return (isXinRange and isYinRange)
		else:
			return True

	def goal_callback(self, msg):
		self.goalPose = msg
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
