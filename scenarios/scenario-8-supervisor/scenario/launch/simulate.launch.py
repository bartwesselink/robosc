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
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'tb3_simulation_launch.py')
            ),
            launch_arguments={'rviz_config_file': os.path.join(get_package_share_directory('scenario'), 'rviz', 'config.rviz')}.items(),
        ),

        Node(
            package='emergency_stop',
            namespace='emergency_stop',
            executable='interface',
            name='emergency_stop'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('supervisor'), 'launch', 'supervisor_with_controller.launch.py')
            ),
        ),
    ])