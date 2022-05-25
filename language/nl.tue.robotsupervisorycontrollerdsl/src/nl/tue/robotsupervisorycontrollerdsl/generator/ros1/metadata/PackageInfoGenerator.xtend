package nl.tue.robotsupervisorycontrollerdsl.generator.ros1.metadata

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.library.PackageHelper

@Singleton
class PackageInfoGenerator {
	@Inject extension PackageHelper
	
	def compilePackageFile(Robot robot, String name)'''
	<?xml version="1.0"?>
	<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
	<package format="3">
	  <name>«name»</name>
	  <version>1.0.0</version>
	  <description>Simple «name».</description>
	  <maintainer email="b.b.a.wesselink@student.tue.nl">b.wesselink</maintainer>
	  <license>TODO: License declaration</license>
	
	  <buildtool_depend>catkin</buildtool_depend>
	
	  <depend>roscpp</depend>
	  <depend>std_msgs</depend>
	  <depend>dynamic_reconfigure</depend>
	  <depend>actionlib</depend>
	  <depend>message_runtime</depend>
	  «FOR packageName : robot.requiredPackages»
	  <depend>«packageName»</depend>
	  «ENDFOR»
	
	  <test_depend>gtest</test_depend>
	  <test_depend>rostest</test_depend>
	</package>
	'''
}