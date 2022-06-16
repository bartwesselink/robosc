package nl.tue.robotsupervisorycontrollerdsl.generator.ros2.communication

import javax.inject.Singleton
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.FieldNames
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data.PlatformTypeGenerator
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

	override initializeField(Action entity, Robot robot, Config config) '''«entity.fieldName» = rclcpp_action::create_client<«entity.links.actionType»>(this, "«entity.topicName»");'''
	override declareField(Action entity, Robot robot, Config config) '''rclcpp_action::Client<«entity.links.actionType»>::SharedPtr «entity.fieldName»;'''
		

	override functions(Action entity, Robot robot, Config config)'''
	void «entity.responseMethod»(const rclcpp_action::ClientGoalHandle<«entity.links.actionType»>::WrappedResult & result) {
		«entity.prepareResult(entity.responseType, robot, 'result')»

		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.responseTransitionName»);
		
		«IF config.writeEventsToLog»
		this->write_to_incoming_log("«entity.responseOutput»");
		«ENDIF»
	}
	
	void «entity.feedbackMethod»(rclcpp_action::ClientGoalHandle<«entity.links.actionType»>::SharedPtr, const std::shared_ptr<const «entity.links.actionType»::Feedback> feedback) {
		«entity.prepareResult(entity.responseType, robot, 'feedback')»

		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.feedbackTransitionName»);
				
		«IF config.writeEventsToLog»
		this->write_to_incoming_log("«entity.feedbackOutput»");
		«ENDIF»
	}
	
	void «entity.callMethod»() {
		if (!this->«entity.fieldName»->wait_for_action_server(1s)) {
			«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.errorTransitionName»);
			return;
		}
		auto goal_msg = «entity.links.actionType»::Goal();

		«entity.compileDataStates(entity.requestType, 'goal_msg', robot, false)»
		
		auto send_options = rclcpp_action::Client<«entity.links.actionType»>::SendGoalOptions();
		send_options.result_callback = std::bind(&Controller::«entity.responseMethod», this, std::placeholders::_1);
		send_options.feedback_callback = std::bind(&Controller::«entity.feedbackMethod», this, std::placeholders::_1, std::placeholders::_2);
		this->«entity.fieldName»->async_send_goal(goal_msg, send_options);
						
		«IF config.writeEventsToLog»
		this->write_to_outgoing_log("«entity.requestOutput»");
		«ENDIF»
	}
		
	void «entity.cancelMethod»() {
		this->«entity.fieldName»->async_cancel_all_goals();
								
		«IF config.writeEventsToLog»
		this->write_to_outgoing_log("«entity.cancelOutput»");
		«ENDIF»
	}
	'''
	
}