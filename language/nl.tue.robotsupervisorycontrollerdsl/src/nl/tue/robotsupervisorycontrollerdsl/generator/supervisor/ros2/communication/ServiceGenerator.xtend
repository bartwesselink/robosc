package nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.communication

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.naming.FieldNames
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data.PlatformTypeGenerator
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.naming.MethodNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractCommunicationTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.remapping.SupervisorMappingNamer

@Singleton
class ServiceGenerator extends AbstractCommunicationTypeGenerator<Service> {
	@Inject extension FieldNames
	@Inject extension PlatformTypeGenerator
	@Inject extension MethodNames
	@Inject extension TransitionNames
	@Inject SupervisorMappingNamer supervisorMappingNamer

	override initializeField(Service entity, Robot robot) '''
	«entity.fieldNameSupervised» = this->create_service<«entity.links.serviceType»>("«entity.topicName(supervisorMappingNamer)»", std::bind(&Supervisor::«entity.handleSupervised», this, std::placeholders::_1, std::placeholders::_2));
	«entity.fieldName» = this->create_client<«entity.links.serviceType»>("«entity.topicName»");
	'''

	override declareField(Service entity, Robot robot) '''
	rclcpp::Service<«entity.links.serviceType»>::SharedPtr «entity.fieldNameSupervised»;
	rclcpp::Client<«entity.links.serviceType»>::SharedPtr «entity.fieldName»;
	std::mutex «entity.mutexLockNameSupervised»;
	«entity.links.serviceType»::Request::SharedPtr «entity.dataHolderNameSupervised»;
	«entity.links.serviceType»::Response::SharedPtr «entity.responseHolderNameSupervised»;
	std::condition_variable «entity.conditionVariableSupervised»;
	bool «entity.responseReadySupervised» = false;
	'''
	
	override functions(Service entity, Robot robot)'''
	// Handle incoming request
	void «entity.handleSupervised»(const std::shared_ptr<«entity.links.serviceType»::Request> request, std::shared_ptr<«entity.links.serviceType»::Response> response) {
		this->«entity.mutexLockNameSupervised».lock();
				
		this->«entity.dataHolderNameSupervised» = request;
		if (!«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.triggerTransitionName»)) {
			this->publish_block("«entity.topicName»");
		}
		this->«entity.dataHolderNameSupervised» = nullptr;

		this->«entity.mutexLockNameSupervised».unlock();
		
		std::unique_lock<std::mutex> lk(this->«entity.mutexLockNameSupervised»);
	    «entity.conditionVariableSupervised».wait(lk, [this]{return this->«entity.responseReadySupervised»;});
	    
	    response.reset();
	    
	    this->«entity.responseReadySupervised» = false;
	}

	// Execute if possible
	bool «entity.responseMethod»(rclcpp::Client<«entity.links.serviceType»>::SharedFuture future) {
		std::shared_ptr<«entity.links.serviceType»_Response> result = future.get();

		«entity.prepareResult(entity.responseType, robot, 'result')»
		
		// Call engine function
		«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.responseTransitionName»);
		
		// Pass on data to the service response
		this->«entity.responseHolderNameSupervised» = result;
		this->«entity.responseReadySupervised» = true;
		«entity.conditionVariableSupervised».notify_one();
		
		this->execute_all_silent();
				
		return true;
	}
	
	
	void «entity.callMethod»() {
		«entity.fieldName»->async_send_request(this->«entity.dataHolderNameSupervised», std::bind(&Supervisor::«entity.responseMethod», this, std::placeholders::_1));
	}
	'''
}