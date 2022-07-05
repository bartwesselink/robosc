import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	remappings = [
		("/correction", "/correction_p7PRNQJ53OH1G"),
		("/no_line", "/no_line_pNTO0V7A8NW2W"),
		("/scan", "/scan_pV4REPE6TL8AC"),
		("/simple_movement", "/simple_movement_p3KAN9HMKYC27"),
		("/stop", "/stop_pS5SS07H3L3YR"),
		("/continue", "/continue_p0GQQSL4MR92X"),
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
