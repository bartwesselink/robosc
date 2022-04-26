package nl.tue.robotsupervisorycontrollerdsl.generator.ros2.metadata

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.library.PackageHelper

@Singleton
class PackageInfoGenerator {
	@Inject extension PackageHelper
	
	def compilePackageFile(Robot robot)'''
	<?xml version="1.0"?>
	<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
	<package format="3">
	  <name>controller</name>
	  <version>1.0.0</version>
	  <description>Simple controller.</description>
	  <maintainer email="noreply@example.com">TODO: Maintainer declaration</maintainer>
	  <license>TODO: License declaration</license>
	
	  <buildtool_depend>ament_cmake</buildtool_depend>
	
	  <test_depend>ament_lint_auto</test_depend>
	  <test_depend>ament_lint_common</test_depend>
	
	  <depend>rclcpp</depend>
	  <depend>rclcpp_action</depend>
	  <depend>std_msgs</depend>
	  «FOR packageName : robot.requiredPackages»
	  <depend>«packageName»</depend>
	  «ENDFOR»
	
	  <export>
	    <build_type>ament_cmake</build_type>
	  </export>
	</package>
	'''
}