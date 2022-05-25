package nl.tue.robotsupervisorycontrollerdsl.generator.ros1.metadata

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.library.PackageHelper
import javax.inject.Inject

@Singleton
class CMakeGenerator {
	@Inject extension PackageHelper
	
	def compileCMakeFile(Robot robot, String name)'''
	cmake_minimum_required(VERSION 3.0.2)
	project(«name»)
	
	# Default to C++14
	if(NOT CMAKE_CXX_STANDARD)
	  set(CMAKE_CXX_STANDARD 14)
	endif()
	
	if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	  add_compile_options(-Wall -Wextra -Wpedantic  -Wno-unused-parameter -Wno-pedantic)
	endif()
	
	find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization actionlib_msgs actionlib «robot.requiredPackages.join(' ')»)
	
	include_directories(${catkin_INCLUDE_DIRS})
	link_directories(${catkin_LIBRARY_DIRS})
	
	catkin_package()
	
	add_library(controller_engine include/controller/controller_engine.c)
	
	add_executable(interface src/node.cpp)
	add_dependencies(interface controller_engine)
	add_dependencies(interface ${catkin_EXPORTED_TARGETS})
	
	add_definitions(-DMAX_NUM_EVENTS=0 -DEVENT_OUTPUT=1)
	
	target_link_libraries(interface controller_engine)
	target_link_libraries(interface ${catkin_LIBRARIES})
	
	install(TARGETS interface
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
	
	install(DIRECTORY launch
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)
	'''
}