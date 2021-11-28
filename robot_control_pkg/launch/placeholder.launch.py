import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(package='robot_control_pkg', executable='placeholder_control',
         output='screen'),
        Node(package='robot_control_pkg', executable='placeholder_estimator',
        	 output='screen'),
    ])

