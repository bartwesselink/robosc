package nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.metadata

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.remapping.SupervisorMappingService
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.DefaultIdentifierNamer
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.remapping.SupervisorMappingNamer

@Singleton
class LaunchFileGenerator {
	@Inject SupervisorMappingService supervisorMappingService
	@Inject DefaultIdentifierNamer defaultIdentifierNamer
	@Inject SupervisorMappingNamer supervisorMappingNamer
	
	def compileLaunchFile(Robot robot, Config config)'''
	import os
	
	from ament_index_python.packages import get_package_share_directory
	from launch import LaunchDescription
	from launch.actions import IncludeLaunchDescription
	from launch_ros.actions import Node
	from launch.launch_description_sources import PythonLaunchDescriptionSource
	
	def generate_launch_description():
		remappings = [
			«FOR remapped : robot.uniqueRemappings»
			("«remapped.key»", "«remapped.value»"),
			«ENDFOR»
		]

		return LaunchDescription([
			«IF config.supervisor?.controller?.launch !== null»
			IncludeLaunchDescription(
				PythonLaunchDescriptionSource(
					os.path.join(get_package_share_directory('«config.supervisor?.controller?.launch?.packageName»'), 'launch', '«config.supervisor?.controller?.launch?.file»')
				),
				launch_arguments={'remappings': remappings}
			),
	        «ENDIF»
	
			«IF config.supervisor?.controller?.run !== null»
			Node(
				package='«config.supervisor?.controller?.run?.packageName»',
				«IF config.supervisor?.controller?.run?.namespace !== null»namespace='«config.supervisor?.controller?.run?.namespace»',«ENDIF»
				executable='«config.supervisor?.controller?.run?.executable»',
				«IF config.supervisor?.controller?.run?.name !== null»name='«config.supervisor?.controller?.run?.name»',«ENDIF»
				remappings=remappings
			),
	        «ENDIF»
	
	        Node(
	            package='supervisor',
	            namespace='supervisor',
	            executable='interface',
	            name='supervisor'
	        ),
	    ])
	'''
	
	private def uniqueRemappings(Robot robot) {
		val result = newArrayList
		val seen = newArrayList
		val all = supervisorMappingService.getAllRemappedCommunicationTypes(robot)
		
		for (remapped : all) {
			val original = defaultIdentifierNamer.name(remapped)
			val transformed = supervisorMappingNamer.name(remapped)
			
			if (!seen.contains(original)) {
				result.add(original -> transformed)
			
				seen.add(original)
			}
		}
		
		return result
	}
}