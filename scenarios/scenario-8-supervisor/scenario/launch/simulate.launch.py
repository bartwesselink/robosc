#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': os.path.join(get_package_share_directory('scenario'), 'worlds', 'lines.world')}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
            ),
        ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', 'True'],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        Node(
            package='emergency_stop',
            namespace='emergency_stop',
            executable='interface',
            name='emergency_stop'
        ),

        Node(
            package='line_detector',
            namespace='line_detector',
            executable='interface',
            name='line_detector'
        ),

        Node(
            package='simple_movement',
            namespace='simple_movement',
            executable='interface',
            name='simple_movement'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('supervisor'), 'launch', 'supervisor_with_controller.launch.py')
            ),
        ),
    ])