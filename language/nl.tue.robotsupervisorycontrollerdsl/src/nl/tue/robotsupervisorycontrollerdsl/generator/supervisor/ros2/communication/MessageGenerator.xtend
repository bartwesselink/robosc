package nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.communication

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.naming.FieldNames
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data.PlatformTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageTo
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.naming.MethodNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractCommunicationTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.remapping.SupervisorMappingNamer
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.logging.LogOutputConverter

@Singleton
class MessageGenerator extends AbstractCommunicationTypeGenerator<Message> {
	@Inject extension FieldNames
	@Inject extension PlatformTypeGenerator
	@Inject extension MethodNames
	@Inject extension TransitionNames
	@Inject SupervisorMappingNamer supervisorMappingNamer
	@Inject extension LogOutputConverter

	override initializeField(Message entity, Robot robot, Config config) {
		// Allow all uncontrollable events just to happen
		if (entity.direction instanceof MessageFrom) {
			return '''
			«entity.fieldNameSupervised» = this->create_publisher<«entity.type.messageType(entity.links)»>("«entity.topicName(supervisorMappingNamer)»", 10);
			«entity.fieldName» = this->create_subscription<«entity.type.messageType(entity.links)»>("«entity.topicName»", 10, std::bind(&Supervisor::«entity.callbackMethod», this, std::placeholders::_1));
			'''
		} else if (entity.direction instanceof MessageTo) {
			// The supervisor should create a subscription for controllable events
			return '''
			«entity.fieldNameSupervised» = this->create_subscription<«entity.type.messageType(entity.links)»>("«entity.topicName(supervisorMappingNamer)»", 10, std::bind(&Supervisor::«entity.callbackMethodSupervised», this, std::placeholders::_1));
			«entity.fieldName» = this->create_publisher<«entity.type.messageType(entity.links)»>("«entity.topicName»", 10);
			'''
		}
	}
	
	override declareField(Message entity, Robot robot, Config config) {
		if (entity.direction instanceof MessageFrom) {
			return '''
			rclcpp::Publisher<«entity.type.messageType(entity.links)»>::SharedPtr «entity.fieldNameSupervised»;
			rclcpp::Subscription<«entity.type.messageType(entity.links)»>::SharedPtr «entity.fieldName»;
			'''
		} else if (entity.direction instanceof MessageTo) {
			// The supervisor should create a subscription for controllable events
			return '''
			rclcpp::Subscription<«entity.type.messageType(entity.links)»>::SharedPtr «entity.fieldNameSupervised»;
			rclcpp::Publisher<«entity.type.messageType(entity.links)»>::SharedPtr «entity.fieldName»;
			std::mutex «entity.mutexLockNameSupervised»;
			«entity.type.messageType(entity.links)»::SharedPtr «entity.dataHolderNameSupervised»;
			'''
		}
	}
	
	override functions(Message entity, Robot robot, Config config)'''
	«IF entity.direction instanceof MessageFrom»
	void «entity.callbackMethod»(const «entity.type.messageType(entity.links)»::SharedPtr msg) {
		«entity.prepareResult(entity.type, robot, 'msg')»
		
		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.eventTransitionName»);
		
		// Forward event
		this->«entity.fieldNameSupervised»->publish(*msg);

		this->execute_all_silent();
		
		«IF config.writeEventsToLog»
		this->write_to_incoming_log("«entity.responseOutput»");
		«ENDIF»
	}
	«ENDIF»
	
	«IF entity.direction instanceof MessageTo»
	void «entity.callbackMethodSupervised»(const «entity.type.messageType(entity.links)»::SharedPtr msg) {
		// Call engine function
		this->«entity.mutexLockNameSupervised».lock();
		
		this->«entity.dataHolderNameSupervised» = msg;
		if (!«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.eventTransitionName»)) {
			this->publish_block("«entity.topicName»");
		}
		this->«entity.dataHolderNameSupervised» = nullptr;

		this->«entity.mutexLockNameSupervised».unlock();

		this->execute_all_silent();
	}

	void «entity.callMethod»() {
		this->«entity.fieldName»->publish(*this->«entity.dataHolderNameSupervised»);

		«IF config.writeEventsToLog»
		this->write_to_outgoing_log("«entity.requestOutput»");
		«ENDIF»
	}
	«ENDIF»
	'''
}