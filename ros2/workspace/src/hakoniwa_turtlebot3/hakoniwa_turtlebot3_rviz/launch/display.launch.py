#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('hakoniwa_turtlebot3_rviz'),
        'rviz',
        'hakoniwa_rviz_tb3b.rviz')
    robot_state_path = os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'launch',
        'turtlebot3_state_publisher.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_path)
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])
