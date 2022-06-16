package nl.tue.robotsupervisorycontrollerdsl.generator.ros1.communication

import javax.inject.Singleton
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.FieldNames
import nl.tue.robotsupervisorycontrollerdsl.generator.ros1.data.PlatformTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.MethodNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.DataPlantHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractCommunicationTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.logging.LogOutputConverter

@Singleton
class ActionGenerator extends AbstractCommunicationTypeGenerator<Action> {
	@Inject extension FieldNames
	@Inject extension PlatformTypeGenerator
	@Inject extension MethodNames
	@Inject extension TransitionNames
	@Inject extension DataPlantHelper
	@Inject extension LogOutputConverter

	override initializeField(Action entity, Robot robot, Config config) ''''''
	override declareField(Action entity, Robot robot, Config config) '''actionlib::SimpleActionClient<«entity.links.actionType»Action>«entity.fieldName»;'''
	def constructorInvocation(Action entity, Robot robot) '''«entity.fieldName»("«entity.topicName»", false)'''

	override functions(Action entity, Robot robot, Config config)'''
	void «entity.responseMethod»(const actionlib::SimpleClientGoalState& state, const «entity.links.actionType»ResultConstPtr& result) {
		«entity.prepareResult(entity.responseType, robot, 'result.result')»

		// Call engine function		        
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.responseTransitionName»);
		} else {
			«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.errorTransitionName»);
		}
				
		«IF config.writeEventsToLog»
		this->write_to_incoming_log("«entity.responseOutput»");
		«ENDIF»
	}
	
	void «entity.feedbackMethod»(const «entity.links.actionType»FeedbackConstPtr& feedback) {
		«entity.prepareResult(entity.responseType, robot, 'feedback')»
		
		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.feedbackTransitionName»);
				
		«IF config.writeEventsToLog»
		this->write_to_incoming_log("«entity.feedbackOutput»");
		«ENDIF»
	}
	
	void «entity.callMethod»() {
		«entity.fieldName».waitForServer();
		«entity.links.actionType»Goal goal_msg;

		«entity.compileDataStates(entity.requestType, 'goal_msg', robot, false)»
		
		«entity.fieldName».sendGoal(goal_msg, boost::bind(&Controller::«entity.responseMethod», this, _1, _2), actionlib::SimpleActionClient<«entity.links.actionType»Action>::SimpleActiveCallback(), boost::bind(&Controller::«entity.feedbackMethod», this, _1));
				
		«IF config.writeEventsToLog»
		this->write_to_outgoing_log("«entity.requestOutput»");
		«ENDIF»
	}
			
	void «entity.cancelMethod»() {
		this->«entity.fieldName».cancelAllGoals();

		«IF config.writeEventsToLog»
		this->write_to_outgoing_log("«entity.cancelOutput»");
		«ENDIF»
	}
	'''
	
}