package nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.communication

import javax.inject.Singleton
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.naming.FieldNames
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data.PlatformTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.naming.MethodNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractCommunicationTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.remapping.SupervisorMappingNamer
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.logging.LogOutputConverter

@Singleton
class ActionGenerator extends AbstractCommunicationTypeGenerator<Action> {
	@Inject extension FieldNames
	@Inject extension PlatformTypeGenerator
	@Inject extension MethodNames
	@Inject extension TransitionNames
	@Inject SupervisorMappingNamer supervisorMappingNamer
	@Inject extension LogOutputConverter

	override initializeField(Action entity, Robot robot, Config config) '''
	«entity.fieldNameSupervised» = rclcpp_action::create_server<«entity.links.actionType»>(
	      this,
	      "«entity.topicName(supervisorMappingNamer)»",
	      std::bind(&Supervisor::«entity.handleGoalSupervised», this, std::placeholders::_1, std::placeholders::_2),
	      std::bind(&Supervisor::«entity.handleCancelledSupervised», this, std::placeholders::_1),
	      std::bind(&Supervisor::«entity.handleAcceptedSupervised», this, std::placeholders::_1));
	«entity.fieldName» = rclcpp_action::create_client<«entity.links.actionType»>(this, "«entity.topicName»");
	'''
	override declareField(Action entity, Robot robot, Config config) '''
	rclcpp_action::Client<«entity.links.actionType»>::SharedPtr «entity.fieldName»;
	rclcpp_action::Server<«entity.links.actionType»>::SharedPtr «entity.fieldNameSupervised»;
	std::mutex «entity.mutexLockNameSupervised»;
	std::shared_ptr<«entity.links.actionType»::Goal> «entity.dataHolderNameSupervised»;
	std::shared_ptr<rclcpp_action::ServerGoalHandle<«entity.links.actionType»>> «entity.goalHandleNameSupervised»;
	'''

	override functions(Action entity, Robot robot, Config config)'''
	// Act as proxy server
	rclcpp_action::GoalResponse «entity.handleGoalSupervised»(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const «entity.links.actionType»::Goal> goal) {
		this->«entity.mutexLockNameSupervised».lock();
		
		auto transformed = std::make_shared<«entity.links.actionType»::Goal>(*goal);
		this->«entity.dataHolderNameSupervised» = transformed;
	
		this->«entity.mutexLockNameSupervised».unlock();

	    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}
	
	rclcpp_action::CancelResponse «entity.handleCancelledSupervised»(const std::shared_ptr<rclcpp_action::ServerGoalHandle<«entity.links.actionType»>> goal_handle) {
	    return this->«entity.cancelMethod»();
	}
	
	void «entity.handleAcceptedSupervised»(const std::shared_ptr<rclcpp_action::ServerGoalHandle<«entity.links.actionType»>> goal_handle) {
		this->«entity.goalHandleNameSupervised» = goal_handle;
		if (!«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.triggerTransitionName»)) {
			this->publish_block("«entity.topicName»");
		}
	}
	
	// Handle events from actual action server
	void «entity.responseMethod»(const rclcpp_action::ClientGoalHandle<«entity.links.actionType»>::WrappedResult & result) {
		«entity.prepareResult(entity.responseType, robot, 'result')»
		
		this->«entity.mutexLockNameSupervised».lock();
		if (this->«entity.dataHolderNameSupervised» != nullptr) {
			auto transformed = std::make_shared<«entity.links.actionType»::Result>(*result.result);

			switch (result.code) {
				case rclcpp_action::ResultCode::SUCCEEDED:
					this->«entity.goalHandleNameSupervised»->succeed(transformed);
					break;
				case rclcpp_action::ResultCode::ABORTED:
					this->«entity.goalHandleNameSupervised»->abort(transformed);
					break;
				case rclcpp_action::ResultCode::CANCELED:
					this->«entity.goalHandleNameSupervised»->canceled(transformed);
					break;
				default:
					this->«entity.goalHandleNameSupervised»->abort(transformed);
					break;
		    }
			
			this->«entity.dataHolderNameSupervised» = nullptr;
			this->«entity.goalHandleNameSupervised» = nullptr;
		}
		this->«entity.mutexLockNameSupervised».unlock();
				
		
		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.responseTransitionName»);
		this->execute_all_silent();

		«IF config.writeEventsToLog»
		this->write_to_incoming_log("«entity.responseOutput»");
		«ENDIF»
	}
	
	void «entity.feedbackMethod»(rclcpp_action::ClientGoalHandle<«entity.links.actionType»>::SharedPtr handle, const std::shared_ptr<const «entity.links.actionType»::Feedback> feedback) {
		«entity.prepareResult(entity.responseType, robot, 'feedback')»
		
		auto transformed = std::make_shared<«entity.links.actionType»::Feedback>(*feedback);
		this->«entity.goalHandleNameSupervised»->publish_feedback(transformed);
		
		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.feedbackTransitionName»);
		this->execute_all_silent();
		
		«IF config.writeEventsToLog»
		this->write_to_incoming_log("«entity.responseOutput»");
		«ENDIF»
	}
	
	void «entity.callMethod»() {
		if (!this->«entity.fieldName»->wait_for_action_server(1s)) {
			«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.errorTransitionName»);
			return;
		}

		auto send_options = rclcpp_action::Client<«entity.links.actionType»>::SendGoalOptions();
		send_options.result_callback = std::bind(&Supervisor::«entity.responseMethod», this, std::placeholders::_1);
		send_options.feedback_callback = std::bind(&Supervisor::«entity.feedbackMethod», this, std::placeholders::_1, std::placeholders::_2);
		this->«entity.fieldName»->async_send_goal(*this->«entity.dataHolderNameSupervised», send_options);
		
		«IF config.writeEventsToLog»
		this->write_to_outgoing_log("«entity.requestOutput»");
		«ENDIF»
	}
		
	rclcpp_action::CancelResponse «entity.cancelMethod»() {
		auto result = this->«entity.fieldName»->async_cancel_all_goals().get();
		
		if (result->return_code == 0) {
			return rclcpp_action::CancelResponse::ACCEPT;
		} else {
			return rclcpp_action::CancelResponse::REJECT;
		}
	}
	'''
	
}