package nl.tue.robotsupervisorycontrollerdsl.generator.ros1.communication

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.FieldNames
import nl.tue.robotsupervisorycontrollerdsl.generator.ros1.data.PlatformTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageTo
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.MethodNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.DataPlantHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractCommunicationTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.logging.LogOutputConverter

@Singleton
class MessageGenerator extends AbstractCommunicationTypeGenerator<Message> {
	@Inject extension FieldNames
	@Inject extension PlatformTypeGenerator
	@Inject extension MethodNames
	@Inject extension TransitionNames
	@Inject extension DataPlantHelper
	@Inject extension LogOutputConverter

	override initializeField(Message entity, Robot robot, Config config) {
		if (entity.direction instanceof MessageFrom) {
			return '''«entity.fieldName» = node.subscribe("«entity.topicName»", 10, &Controller::«entity.callbackMethod», this);'''
		} else if (entity.direction instanceof MessageTo) {
			return '''«entity.fieldName» = node.advertise<«entity.type.messageType(entity.links)»>("«entity.topicName»", 10);'''
		}
	}
	
	override declareField(Message entity, Robot robot, Config config) {
		return '''ros::«IF entity.direction instanceof MessageFrom»Subscriber«ELSE»Publisher«ENDIF» «entity.fieldName»;'''
	}
	
	override functions(Message entity, Robot robot, Config config)'''
	«IF entity.direction instanceof MessageFrom»
	void «entity.callbackMethod»(const «entity.type.messageType(entity.links)»::ConstPtr& msg) {
		«entity.prepareResult(entity.type, robot, 'msg')»
		
		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.eventTransitionName»);
				
		«IF config.writeEventsToLog»
		this->write_to_incoming_log("«entity.responseOutput»");
		«ENDIF»
	}
	«ENDIF»
	
	
	«IF entity.direction instanceof MessageTo»
	void «entity.callMethod»() {
		auto value = «entity.type.messageType(entity.links)»();
		
		«entity.compileDataStates(entity.type, 'value', robot, false)»
		
		this->«entity.fieldName».publish(value);
				
		«IF config.writeEventsToLog»
		this->write_to_outgoing_log("«entity.requestOutput»");
		«ENDIF»
	}
	«ENDIF»
	'''
}