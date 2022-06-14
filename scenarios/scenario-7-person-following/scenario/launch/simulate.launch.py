#!/usr/bin/env python3
# Based on Gazebo Empty World launch file
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    yolox_ros_share_dir = get_package_share_directory('yolox_ros_py')
    yolox_onnx = Node(
        package="yolox_ros_py", executable="yolox_onnx",output="screen",
        parameters=[
            {"input_shape/width": 416},
            {"input_shape/height": 416},

            {"with_p6" : False},
            {"model_path" : yolox_ros_share_dir+"/yolox_nano.onnx"},
            {"conf" : 0.3},
            {"sensor_qos_mode" : True},
        ],
        remappings=[
            ("/image_raw", "/camera/image_raw"),
        ],
    )

    return LaunchDescription([
        yolox_onnx,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': os.path.join(get_package_share_directory('scenario'), 'worlds', 'person.world')}.items(),
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

        # Node(
        #     package='controller',
        #     namespace='controller',
        #     executable='interface',
        #     name='controller'
        # ),
    ])