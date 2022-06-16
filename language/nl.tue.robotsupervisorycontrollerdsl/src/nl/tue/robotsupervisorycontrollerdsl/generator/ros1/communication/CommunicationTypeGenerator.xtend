package nl.tue.robotsupervisorycontrollerdsl.generator.ros1.communication

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config

@Singleton
class CommunicationTypeGenerator {
	@Inject MessageGenerator messageGenerator
	@Inject ServiceGenerator serviceGenerator
	@Inject ActionGenerator actionGenerator
	
	def compileCommunicationFieldDefinitions(Robot robot, Config config)'''
	«FOR communication : ModelHelper.findWithinRobot(robot, CommunicationType)»
	«communication.declareField(robot, config)»
	«ENDFOR»
	'''
	
	def compileCommunicationFieldInitializations(Robot robot, Config config)'''
	«FOR communication : ModelHelper.findWithinRobot(robot, CommunicationType)»
	«communication.initializeField(robot, config)»
	«ENDFOR»
	'''
	
	def compileCommunicationFunctions(Robot robot, Config config)'''
	«FOR communication : ModelHelper.findWithinRobot(robot, CommunicationType)»
	«communication.functions(robot, config)»
	«ENDFOR»
	'''
	
	def constructorInvocations(Robot robot) {
		return ModelHelper.findWithinRobot(robot, CommunicationType)
			.filter(Action)
			.map[it.constructorInvocation(robot)]
	}
	
	private def dispatch initializeField(Message entity, Robot robot, Config config) '''«messageGenerator.initializeField(entity, robot, config)»'''
	private def dispatch initializeField(Service entity, Robot robot, Config config) '''«serviceGenerator.initializeField(entity, robot, config)»'''
	private def dispatch initializeField(Action entity, Robot robot, Config config) '''«actionGenerator.initializeField(entity, robot, config)»'''
	
	private def dispatch declareField(Message entity, Robot robot, Config config) '''«messageGenerator.declareField(entity, robot, config)»'''
	private def dispatch declareField(Service entity, Robot robot, Config config) '''«serviceGenerator.declareField(entity, robot, config)»'''
	private def dispatch declareField(Action entity, Robot robot, Config config) '''«actionGenerator.declareField(entity, robot, config)»'''
	
	private def dispatch functions(Message entity, Robot robot, Config config) '''«messageGenerator.functions(entity, robot, config)»'''
	private def dispatch functions(Service entity, Robot robot, Config config) '''«serviceGenerator.functions(entity, robot, config)»'''
	private def dispatch functions(Action entity, Robot robot, Config config) '''«actionGenerator.functions(entity, robot, config)»'''
	
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