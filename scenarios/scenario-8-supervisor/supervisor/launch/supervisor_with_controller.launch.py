import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	remappings = [
		("/correction", "/correction_pD03NAUQLGW9U"),
		("/no_line", "/no_line_pPWS7VOXLHSV1"),
		("/scan", "/scan_pLGJ50J0RL33J"),
		("/simple_movement", "/simple_movement_pNGAJWRO2BE69"),
		("/stop", "/stop_pO55LCZMBJI3R"),
		("/continue", "/continue_pE153PVY940FM"),
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
