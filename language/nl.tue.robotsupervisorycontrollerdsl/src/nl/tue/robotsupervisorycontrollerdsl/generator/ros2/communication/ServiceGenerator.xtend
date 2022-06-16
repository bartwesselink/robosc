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
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.logging.LogOutputConverter

@Singleton
class ServiceGenerator extends AbstractCommunicationTypeGenerator<Service> {
	@Inject extension FieldNames
	@Inject extension PlatformTypeGenerator
	@Inject extension MethodNames
	@Inject extension DataPlantHelper
	@Inject extension TransitionNames
	@Inject extension LogOutputConverter

	override initializeField(Service entity, Robot robot, Config config) '''«entity.fieldName» = this->create_client<«entity.links.serviceType»>("«entity.topicName»");'''

	override declareField(Service entity, Robot robot, Config config) '''rclcpp::Client<«entity.links.serviceType»>::SharedPtr «entity.fieldName»;'''
	
	override functions(Service entity, Robot robot, Config config)'''
	bool «entity.responseMethod»(rclcpp::Client<«entity.links.serviceType»>::SharedFuture future) {
		std::shared_ptr<«entity.links.serviceType»_Response> result = future.get();

		«entity.prepareResult(entity.responseType, robot, 'result')»

		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.responseTransitionName»);
										
		«IF config.writeEventsToLog»
		this->write_to_incoming_log("«entity.responseOutput»");
		«ENDIF»
		
		return true;
	}
	
	void «entity.callMethod»() {
		auto request = std::make_shared<«entity.links.serviceType»::Request>();
		
		«entity.compileDataStates(entity.requestType, 'request', robot, true)»
		
		using ServiceResponseFuture = rclcpp::Client<«entity.links.serviceType»>::SharedFutureWithRequest;
		auto result = «entity.fieldName»->async_send_request(request, std::bind(&Controller::«entity.responseMethod», this, std::placeholders::_1));
										
		«IF config.writeEventsToLog»
		this->write_to_outgoing_log("«entity.requestOutput»");
		«ENDIF»
	}
	'''
}