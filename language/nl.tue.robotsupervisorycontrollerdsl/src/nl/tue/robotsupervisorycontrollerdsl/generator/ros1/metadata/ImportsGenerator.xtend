package nl.tue.robotsupervisorycontrollerdsl.generator.ros1.metadata

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.library.PackageHelper
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.ros1.data.PlatformTypeGenerator
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
	#include <stdbool.h>
	#include <actionlib/client/simple_action_client.h>
	
	#include <ros/ros.h>
	#include <actionlib/client/simple_action_client.h>
	#include "std_msgs/String.h"
	#include "std_msgs/Bool.h"
	#include "std_msgs/Empty.h"
	#include "std_msgs/Int16.h"
	#include "std_msgs/Float32.h"
	«FOR importName : robot.getAllImports(typeGenerator)»
	#include "«importName».h"
	«ENDFOR»
	#include <cinttypes>
	
	extern "C" {
	    #include "../include/controller/«CifSynthesisTool.codePrefix»_engine.h"
	}
	
	using namespace std::chrono_literals;
	'''
}