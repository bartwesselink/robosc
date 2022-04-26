package nl.tue.robotsupervisorycontrollerdsl.generator.ros1.communication

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import javax.inject.Inject

@Singleton
class CommunicationTypeGenerator {
	@Inject MessageGenerator messageGenerator
	@Inject ServiceGenerator serviceGenerator
	@Inject ActionGenerator actionGenerator
	
	def compileCommunicationFieldDefinitions(Robot robot)'''
	«FOR communication : ModelHelper.findWithinRobot(robot, CommunicationType)»
	«communication.declareField(robot)»
	«ENDFOR»
	'''
	
	def compileCommunicationFieldInitializations(Robot robot)'''
	«FOR communication : ModelHelper.findWithinRobot(robot, CommunicationType)»
	«communication.initializeField(robot)»
	«ENDFOR»
	'''
	
	def compileCommunicationFunctions(Robot robot)'''
	«FOR communication : ModelHelper.findWithinRobot(robot, CommunicationType)»
	«communication.functions(robot)»
	«ENDFOR»
	'''
	
	def constructorInvocations(Robot robot) {
		return ModelHelper.findWithinRobot(robot, CommunicationType)
			.filter(Action)
			.map[it.constructorInvocation(robot)]
	}
	
	private def dispatch initializeField(Message entity, Robot robot) '''«messageGenerator.initializeField(entity, robot)»'''
	private def dispatch initializeField(Service entity, Robot robot) '''«serviceGenerator.initializeField(entity, robot)»'''
	private def dispatch initializeField(Action entity, Robot robot) '''«actionGenerator.initializeField(entity, robot)»'''
	
	private def dispatch declareField(Message entity, Robot robot) '''«messageGenerator.declareField(entity, robot)»'''
	private def dispatch declareField(Service entity, Robot robot) '''«serviceGenerator.declareField(entity, robot)»'''
	private def dispatch declareField(Action entity, Robot robot) '''«actionGenerator.declareField(entity, robot)»'''
	
	private def dispatch functions(Message entity, Robot robot) '''«messageGenerator.functions(entity, robot)»'''
	private def dispatch functions(Service entity, Robot robot) '''«serviceGenerator.functions(entity, robot)»'''
	private def dispatch functions(Action entity, Robot robot) '''«actionGenerator.functions(entity, robot)»'''
	
	private def dispatch CharSequence constructorInvocation(Message entity, Robot robot) {
		return null
	}
	private def dispatch CharSequence constructorInvocation(Service entity, Robot robot) {
		return null
	}
	private def dispatch CharSequence constructorInvocation(Action entity, Robot robot) {
		return actionGenerator.constructorInvocation(entity, robot)
	}
}