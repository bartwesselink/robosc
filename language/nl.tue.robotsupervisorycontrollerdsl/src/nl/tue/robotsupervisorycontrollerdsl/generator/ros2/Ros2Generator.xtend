package nl.tue.robotsupervisorycontrollerdsl.generator.ros2

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.common.GeneratorInterface
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import org.eclipse.xtext.generator.IFileSystemAccess2
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.EnumDataTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine.ShuffleHelperGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine.InitializationGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine.EventExecutionGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.communication.CommunicationTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.metadata.CMakeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.metadata.PackageInfoGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine.CommunicationTypeHookGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.metadata.ImportsGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.EliminationHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.FileHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data.PlatformTypeGenerator

@Singleton
class Ros2Generator implements GeneratorInterface {
	@Inject extension EnumDataTypeGenerator
	@Inject extension EventExecutionGenerator
	@Inject extension CommunicationTypeGenerator
	@Inject extension CMakeGenerator
	@Inject extension PackageInfoGenerator
	@Inject extension ImportsGenerator
	@Inject ShuffleHelperGenerator shuffleHelperGenerator
	@Inject extension CommunicationTypeHookGenerator
	@Inject extension EliminationHelper
	@Inject InitializationGenerator initializationGenerator
	@Inject CifSynthesisTool cifSynthesisTool
	@Inject PlatformTypeGenerator platformTypeGenerator

	override generate(Robot robot, IFileSystemAccess2 fileSystemAccess) {
		val base = '''«robot.name»/ros2/controller'''
		val fileName = '''«base»/src/controller_member_function.cpp'''

		fileSystemAccess.generateFile(fileName, robot.controller)
		fileSystemAccess.generateFile(base + '/package.xml', robot.compilePackageFile)
		fileSystemAccess.generateFile(base + '/CMakeLists.txt', robot.compileCMakeFile)
		
		val absolutePath = FileHelper.findAbsolutePath(robot.name, fileSystemAccess, robot.eResource.resourceSet)
		cifSynthesisTool.copyOutputFiles(fileSystemAccess, absolutePath, '''«base»/include/controller/''')
	}

	def controller(Robot robot) '''
		«robot.determineRequiredImports»
	
		// Utility functions
		«shuffleHelperGenerator.generateShuffleFunction»
		
		«robot.compileCodeOnlyVariables»
		
		class Controller : public rclcpp::Node {
		public:	
			// Enum conversions
			«FOR component : robot.definitions.filter(EnumDataType)»«component.compile(platformTypeGenerator)»«ENDFOR»

			«robot.compileCommunicationFieldDefinitions»
			
			Controller() : Node("controller") {
				«robot.compileCommunicationFieldInitializations»

				timer = this->create_wall_timer(100ms, std::bind(&Controller::tick, this));
				«CifSynthesisTool.codePrefix»_EngineFirstStep();
			}

			«robot.compileCommunicationFunctions»
		private:
			// Heart of the controller
			void tick() {
				«robot.compilePerformEventEngine»
			}
			
			rclcpp::TimerBase::SharedPtr timer;
		};
		
		std::shared_ptr<Controller> node_controller = nullptr;
		
		// Control synthesis engine
		«initializationGenerator.initializeEngineVariables(robot)»
		«robot.compileHooks»
		
		int main(int argc, char *argv[]) {
		    rclcpp::init(argc, argv);
		
		    node_controller = std::make_shared<Controller>();
		
		    rclcpp::spin(node_controller);
		    rclcpp::shutdown();
		
		    return 0;
		}
	'''
}
