package nl.tue.robotsupervisorycontrollerdsl.generator.cif.components

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.data.DataPlantGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.data.InputDefinitionGenerator

@Singleton
class CommunicationTypeGenerator {
	@Inject extension DataPlantGenerator
	@Inject extension PlantNames
	@Inject extension TransitionNames
	@Inject extension InputDefinitionGenerator

	def compile(CommunicationType communicationType, Robot robot)'''
	«communicationType.compilePlant(robot)»
	
	«communicationType.compileDataplant(robot)»
	'''
	
	private def dispatch String compilePlant(Message message, Robot robot)'''
	plant «message.plantName»:
		«IF message.direction instanceof MessageFrom»un«ENDIF»controllable «message.transitionName»;
		«IF message.direction instanceof MessageFrom»
		«message.type.inputs(robot, message, 'response')»
		«ENDIF»
		
		location:
		    initial; marked;
		    edge «message.transitionName»;
	end
	'''
	
	private def dispatch String compilePlant(Service service, Robot robot)'''
	plant «service.plantName»:
		controllable c_trigger, c_reset;
		uncontrollable u_response, u_error;
	
		«service.responseType.inputs(robot, service, 'response')»
		
		location idle:
			initial; marked;
			edge c_trigger goto wait_for_response;
		location wait_for_response:
			edge u_response goto ready;
			edge u_error goto error;
		location ready:
			edge c_reset goto idle;
		location error:
			edge c_reset goto idle;
	end
	'''
	
	private def dispatch String compilePlant(Action action, Robot robot)'''
	plant «action.plantName»:
		controllable c_trigger, c_reset, c_cancel;
		uncontrollable u_feedback, u_response, u_error;

		«action.feedbackType.inputs(robot, action, 'feedback')»
		«action.responseType.inputs(robot, action, 'response')»
		
		location idle:
			initial; marked;
			edge c_trigger goto executing;
		location executing:
			edge u_feedback;
			edge u_response goto ready;
			edge u_error goto error;
			edge c_cancel goto idle;
		location ready:
			edge c_reset goto idle;
		location error:
			edge c_reset goto idle;
	end
	'''
}