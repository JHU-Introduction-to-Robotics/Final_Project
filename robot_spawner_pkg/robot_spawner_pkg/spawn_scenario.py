"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
"""
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node

class GoalPublisher(Node):

    def __init__(self,goal_x,goal_y,goal_z):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Pose, '/en613/goal', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.goal_z = float(goal_z)

    def timer_callback(self):
        msg = Pose()
        msg.position.x = self.goal_x
        msg.position.y = self.goal_y
        msg.position.z = self.goal_z
        self.publisher_.publish(msg)

start_positions = [[1.0,-1,0.1],[4.5,-3.25,0.1],[-1.5,0.5,0.1],[3,0.8,0.1]]
goal_positions = [[-1.5,0.5,0.1],[-1.5,-1,0.1],[3.0,-2.5,0.1],[-1.5,-4.25,0.1]]
def main():
    """ Main for spwaning turtlebot node """
    # Get input arguments from user
    argv = sys.argv[1:]

    # Start node
    rclpy.init()
    sdf_file_path = os.path.join(
        get_package_share_directory("robot_spawner_pkg"), "models",
        "basic_robot", "model.sdf")
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    scen_id = int(argv[2])
    start = start_positions[scen_id]
    goal = goal_positions[scen_id]

    print(f"robot_sdf={sdf_file_path}")
    print(f"Start pose={start_positions[scen_id]}")
    # Set data for request
    request = SpawnEntity.Request()
    request.name = argv[0]
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(start[0])
    request.initial_pose.position.y = float(start[1])
    request.initial_pose.position.z = float(start[2])
    node.get_logger().info("Sending service request to `/spawn_entity` (spawning robot)")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())


    marker_sdf_file_path = os.path.join(
        get_package_share_directory("robot_spawner_pkg"), "models",
        "globe", "model.sdf")
    print(f"goal_marker_sdf={marker_sdf_file_path}")
    print(f"Start pose={goal}")
    # Set data for request
    request = SpawnEntity.Request()
    request.name = 'Goal'
    request.xml = open(marker_sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(goal[0])
    request.initial_pose.position.y = float(goal[1])
    request.initial_pose.position.z = float(goal[2])

    node.get_logger().info("Sending service request to `/spawn_entity` (spawning goal)")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down spawner node.")
    node.destroy_node()


    #Start publishing the goal position
    goal_publisher = GoalPublisher(goal[0],goal[1],goal[2])

    rclpy.spin(goal_publisher)

    goal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()