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

	override initializeField(Service entity, Robot robot, Config config) '''«entity.fieldName» = node.serviceClient<«entity.links.serviceType»>("«entity.topicName»");'''

	override declareField(Service entity, Robot robot, Config config) '''ros::ServiceClient «entity.fieldName»;'''

	override functions(Service entity, Robot robot, Config config)'''
	void «entity.callMethod»() {
		«entity.links.serviceType» srv;
		auto request = std::make_shared<«entity.links.serviceType»::Request>();
		
		«entity.compileDataStates(entity.requestType, 'srv.request', robot, false)»

		«IF config.writeEventsToLog»
		this->write_to_outgoing_log("«entity.requestOutput»");
		«ENDIF»
		
		if («entity.fieldName».call(srv)) {		
			«entity.prepareResult(entity.responseType, robot, 'srv.response')»

			«CifSynthesisTool.codePrefix»_EnginePerformEvent(«entity.responseTransitionName»);
					
			«IF config.writeEventsToLog»
			this->write_to_incoming_log("«entity.responseOutput»");
			«ENDIF»
		} else {
			«CifSynthesisTool.codePrefix»EnginePerformEvent(«entity.errorTransitionName»);
		}
	}
	'''
}