import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	remappings = [
		("/correction", "/correction_pTJXJGOJ3LG7I"),
		("/no_line", "/no_line_pWTCO9HEYOG9T"),
		("/simple_movement", "/simple_movement_pEIFRW5LRP7VV"),
		("/stop", "/stop_pZBDPKQYE2EB7"),
		("/continue", "/continue_pK0VB4G3N8KBH"),
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
