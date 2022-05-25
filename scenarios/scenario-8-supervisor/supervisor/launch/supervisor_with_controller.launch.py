import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	remappings = [
		("/clicked_point", "/clicked_point_pT6GTW0MY1FJ7"),
		("/initialpose", "/initialpose_pF0790C71IQRO"),
		("/navigate_to_pose", "/navigate_to_pose_pZQ6KSO70PFQJ"),
		("/stop", "/stop_pQTYU6IWH9SIJ"),
		("/continue", "/continue_pJDTWAIIH21J4"),
	]

	return LaunchDescription([

		Node(
			package='controller',
			namespace='controller',
			executable='interface',
			name='controller',
			remappings=remappings
		),

        Node(
            package='supervisor',
            namespace='supervisor',
            executable='interface',
            name='supervisor'
        ),
    ])
