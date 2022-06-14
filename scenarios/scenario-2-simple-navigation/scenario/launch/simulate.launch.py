#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'tb3_simulation_launch.py')
            ),
            launch_arguments={
                'rviz_config_file': os.path.join(get_package_share_directory('scenario'), 'rviz', 'config.rviz'),
                'world': os.path.join(get_package_share_directory('scenario'), 'worlds', 'room.world'),
                'map': os.path.join(get_package_share_directory('scenario'), 'maps', 'map.yaml')
            }.items(),
        ),

        # Node(
        #     package='controller',
        #     namespace='controller',
        #     executable='interface',
        #     name='controller'
        # ),

        Node(
            package='emergency_stop',
            namespace='emergency_stop',
            executable='interface',
            name='emergency_stop'
        ),
    ])