package nl.tue.robotsupervisorycontrollerdsl.generator.ros2.communication

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.FieldNames
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data.PlatformTypeGenerator
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.DataPlantHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.MethodNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractCommunicationTypeGenerator

@Singleton
class ServiceGenerator extends AbstractCommunicationTypeGenerator<Service> {
	@Inject extension FieldNames
	@Inject extension PlatformTypeGenerator
	@Inject extension MethodNames
	@Inject extension DataPlantHelper
	@Inject extension TransitionNames

	override initializeField(Service entity, Robot robot) '''«entity.fieldName» = this->create_client<«entity.typeSettings.serviceType»>("«entity.topicName»");'''

	override declareField(Service entity, Robot robot) '''rclcpp_action::Client<«entity.typeSettings.actionType»>::SharedPtr «entity.fieldName»;'''
	
	override functions(Service entity, Robot robot)'''
	bool «entity.responseMethod»(rclcpp::Client<«entity.typeSettings.serviceType»>::SharedFuture future) {
		std::shared_ptr<«entity.typeSettings.serviceType»_Response> result = future.get();

		«entity.prepareResult(entity.responseType, robot, 'result')»

		fprintf(stderr, "[debug] Received service answer\n");
		
		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.responseTransitionName»);
		
		return true;
	}
	
	
	void «entity.callMethod»() {
		auto request = std::make_shared<«entity.typeSettings.serviceType»::Request>();
		
		«entity.compileDataStates(entity.requestType, 'request', robot, true)»
		
		using ServiceResponseFuture = rclcpp::Client<«entity.typeSettings.serviceType»>::SharedFutureWithRequest;
		auto result = «entity.fieldName»->async_send_request(request, std::bind(&Controller::«entity.responseMethod», this, std::placeholders::_1));
	}
	'''
}