package nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.common.GeneratorInterface
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import org.eclipse.xtext.generator.IFileSystemAccess2
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.EnumDataTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine.ShuffleHelperGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine.InitializationGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.communication.CommunicationTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.metadata.CMakeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.metadata.PackageInfoGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine.CommunicationTypeHookGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.metadata.ImportsGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.EliminationHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.FileHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data.PlatformTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.info.InfoUtilitiesGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine.SerializationHelperGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.output.OutputCopyUtil
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.metadata.LaunchFileGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine.EventExecutionGenerator

@Singleton
class Ros2SupervisorGenerator implements GeneratorInterface {
	@Inject extension EnumDataTypeGenerator
	@Inject extension CommunicationTypeGenerator
	@Inject extension CMakeGenerator
	@Inject extension PackageInfoGenerator
	@Inject extension ImportsGenerator
	@Inject extension LaunchFileGenerator
	@Inject extension InfoUtilitiesGenerator
	@Inject ShuffleHelperGenerator shuffleHelperGenerator
	@Inject SerializationHelperGenerator serializationHelperGenerator
	@Inject extension CommunicationTypeHookGenerator
	@Inject extension EliminationHelper
	@Inject extension EventExecutionGenerator
	@Inject InitializationGenerator initializationGenerator
	@Inject CifSynthesisTool cifSynthesisTool
	@Inject PlatformTypeGenerator platformTypeGenerator
	@Inject OutputCopyUtil outputCopyUtil

	override generate(Robot robot, IFileSystemAccess2 fileSystemAccess, Config config) {
		val base = '''«robot.name»/supervisor/ros2/supervisor'''
		val fileName = '''«base»/src/member_function.cpp'''
		
		val type = 'supervisor'

		fileSystemAccess.generateFile(fileName, robot.supervisor(config))
		fileSystemAccess.generateFile(base + '/package.xml', robot.compilePackageFile(type))
		fileSystemAccess.generateFile(base + '/CMakeLists.txt', robot.compileCMakeFile(type, true))
		fileSystemAccess.generateFile(base + '/launch/supervisor_with_controller.launch.py', robot.compileLaunchFile(config))
		
		val absolutePath = FileHelper.findAbsolutePath(robot.name, fileSystemAccess, robot.eResource.resourceSet)
		cifSynthesisTool.copyOutputFiles(fileSystemAccess, absolutePath, '''«base»/include/controller/''')
		
		if (config.output.ros2SupervisorNodeLocation !== null) {
			outputCopyUtil.copyDirectory(robot, base, config.output.ros2SupervisorNodeLocation, fileSystemAccess)
		}
	}

	def supervisor(Robot robot, Config config) '''
	«robot.determineRequiredImports»

	// Utility functions
	«shuffleHelperGenerator.generateShuffleFunction»
	
	«IF config.publishStateInformation»
	«serializationHelperGenerator.generateSerializeVectorFunction»
	«ENDIF»
	
	«robot.compileCodeOnlyVariables»
	
	class Supervisor : public rclcpp::Node {
	public:	
		// Enum conversions
		«FOR component : robot.definitions.filter(EnumDataType)»«component.compile(platformTypeGenerator)»«ENDFOR»

		«robot.compileCommunicationFieldDefinitions»
		«IF config.publishStateInformation»
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_information;
		«ENDIF»
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr blocked;
		«robot.compileActivationFields»

		Supervisor() : Node("supervisor") {
			«robot.compileCommunicationFieldInitializations»

			blocked = this->create_publisher<std_msgs::msg::String>("/blocked", 10);
			«IF config.publishStateInformation»
			state_information = this->create_publisher<std_msgs::msg::String>("/state", 10);
			«ENDIF»
			timer = this->create_wall_timer(100ms, std::bind(&Supervisor::tick, this));
			«CifSynthesisTool.codePrefix»_EngineFirstStep();
		}

		«robot.compileCommunicationFunctions»
		
		«IF config.publishStateInformation»
		«robot.compileInfoFunction(platformTypeGenerator)»
		«ENDIF»
	private:
		// Only used to emit state information
		void tick() {
			«IF config.publishStateInformation»
			this->emit_current_state();
			«ENDIF»
		}
		
		void execute_all_silent() {
			«robot.compileSilentEventEngine»
		}
		
		void publish_block(std::string identifier) {
			auto msg = std_msgs::msg::String();
			msg.data = identifier;
			
			this->blocked->publish(msg);
		}
		
		rclcpp::TimerBase::SharedPtr timer;
	};
	
	std::shared_ptr<Supervisor> node = nullptr;
	
	// Control synthesis engine
	«initializationGenerator.initializeEngineVariables(robot)»
	«robot.compileHooks»
	
	int main(int argc, char *argv[]) {
	    rclcpp::init(argc, argv);
	
	    node = std::make_shared<Supervisor>();
	
	    rclcpp::spin(node);
	    rclcpp::shutdown();
	
	    return 0;
	}
	'''
}
