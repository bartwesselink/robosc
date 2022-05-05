package nl.tue.robotsupervisorycontrollerdsl.generator.ros1.communication

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.FieldNames
import nl.tue.robotsupervisorycontrollerdsl.generator.ros1.data.PlatformTypeGenerator
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

	override initializeField(Service entity, Robot robot) '''«entity.fieldName» = node.serviceClient<«entity.typeSettings.serviceType»>("«entity.topicName»");'''

	override declareField(Service entity, Robot robot) '''ros::ServiceClient «entity.fieldName»;'''

	override functions(Service entity, Robot robot)'''
	void «entity.callMethod»() {
		activated_services.push_back("«entity.name»");

		«entity.typeSettings.serviceType» srv;
		auto request = std::make_shared<«entity.typeSettings.serviceType»::Request>();
		
		«entity.compileDataStates(entity.requestType, 'srv.request', robot, false)»
		
		if («entity.fieldName».call(srv)) {		
			received_response_services.push_back("«entity.name»");

			«entity.prepareResult(entity.responseType, robot, 'srv.response')»
			
			fprintf(stderr, "[debug] Received service answer\n");
			
			«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.responseTransitionName»);
		} else {
			«CifSynthesisTool.codePrefix»EnginePerformEvent(«entity.errorTransitionName»);
		}
	}
	'''
}