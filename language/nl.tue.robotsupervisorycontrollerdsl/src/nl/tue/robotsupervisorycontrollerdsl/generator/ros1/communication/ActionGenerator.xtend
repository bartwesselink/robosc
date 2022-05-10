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

@Singleton
class ActionGenerator extends AbstractCommunicationTypeGenerator<Action> {
	@Inject extension FieldNames
	@Inject extension PlatformTypeGenerator
	@Inject extension MethodNames
	@Inject extension TransitionNames
	@Inject extension DataPlantHelper

	override initializeField(Action entity, Robot robot) ''''''
	override declareField(Action entity, Robot robot) '''actionlib::SimpleActionClient<«entity.typeSettings.actionType»Action>«entity.fieldName»;'''
	def constructorInvocation(Action entity, Robot robot) '''«entity.fieldName»("«entity.topicName»", false)'''

	override functions(Action entity, Robot robot)'''
	void «entity.responseMethod»(const actionlib::SimpleClientGoalState& state, const «entity.typeSettings.actionType»ResultConstPtr& result) {
		«entity.prepareResult(entity.responseType, robot, 'result.result')»

		fprintf(stderr, "[debug] Received action response\n");
		
		// Call engine function		        
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			fprintf(stderr, "[debug] Received response from action.\n");
			«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.responseTransitionName»);
		} else {
			fprintf(stderr, "[debug] Received error from action.\n");
			«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.errorTransitionName»);
		}
	}
	
	void «entity.feedbackMethod»(const «entity.typeSettings.actionType»FeedbackConstPtr& feedback) {
		«entity.prepareResult(entity.responseType, robot, 'feedback')»

		fprintf(stderr, "[debug] Received action feedback\n");
		
		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.feedbackTransitionName»);
	}
	
	void «entity.callMethod»() {
		«entity.fieldName».waitForServer();
		«entity.typeSettings.actionType»Goal goal_msg;

		«entity.compileDataStates(entity.requestType, 'goal_msg', robot, false)»
		
		«entity.fieldName».sendGoal(goal_msg, boost::bind(&Controller::«entity.responseMethod», this, _1, _2), actionlib::SimpleActionClient<«entity.typeSettings.actionType»Action>::SimpleActiveCallback(), boost::bind(&Controller::«entity.feedbackMethod», this, _1));
	}
			
	void «entity.cancelMethod»() {
		this->«entity.fieldName».cancelAllGoals();
	}
	'''
	
}