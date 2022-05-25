package nl.tue.robotsupervisorycontrollerdsl.generator.ros2.metadata

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.library.PackageHelper
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data.PlatformTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool

@Singleton
class ImportsGenerator {
	@Inject PlatformTypeGenerator typeGenerator
	@Inject extension PackageHelper
	
	def determineRequiredImports(Robot robot)'''
	#include <chrono>
	#include <memory>
	#include <thread>
	#include <unistd.h>
	#include <functional>
	#include <future>
	#include <memory>
	#include <sstream>
	#include <mutex>
	#include <condition_variable>
	
	#include "rclcpp/rclcpp.hpp"
	#include "rclcpp_action/rclcpp_action.hpp"
	#include "std_msgs/msg/string.hpp"
	#include "std_msgs/msg/bool.hpp"
	#include "std_msgs/msg/empty.hpp"
	#include "std_msgs/msg/int16.hpp"
	#include "std_msgs/msg/float32.hpp"
	«FOR importName : robot.getAllImports(typeGenerator)»
	#include "«importName».hpp"
	«ENDFOR»
	#include <cinttypes>
	
	extern "C" {
	    #include "../include/controller/«CifSynthesisTool.codePrefix»_engine.h"
	}
	
	using namespace std::chrono_literals;
	'''
}