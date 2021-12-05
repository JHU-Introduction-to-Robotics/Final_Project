import math
import rclpy
from time import sleep
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
import numpy as np

class PlaceholderController(Node):

	goal_pose = None

	def __init__(self):
		super().__init__('PlaceholderController')
		self.subscription = self.create_subscription(
			Pose,
			'/en613/state_est',
			self.listener_callback,
			10)
		self.goal_subscriber = self.create_subscription(
														Pose,
														'/en613/goal',
														self.goal_callback,
														10
													   )

		self.publisher_ = self.create_publisher(Twist, '/en613/cmd_vel', 10)


	def listener_callback(self, msg):
		self.publish_cmd()


	def publish_cmd(self):

		msg = Twist()
		msg.linear.x = 0.0
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = 0.0

		self.publisher_.publish(msg)

	def goal_callback(self, msg):
		goal_pose = msg
		#self.get_logger().info('Goal Pose: {0}'.format(goal_pose))

	def print_goal(self, msg):
		print(msg)

def main(args=None):
    rclpy.init(args=args)

    controller = PlaceholderController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
