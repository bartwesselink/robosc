import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	remappings = [
		("/correction", "/correction_pAKO3XD5LXQVY"),
		("/no_line", "/no_line_p2O7AZU3ZS3KR"),
		("/scan", "/scan_pKP7IMHKLB4Q5"),
		("/simple_movement", "/simple_movement_pE3M1TMNNE85C"),
		("/stop", "/stop_pGDVQ0MDSIZEK"),
		("/continue", "/continue_p1C6LQOF7DJST"),
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
