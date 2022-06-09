package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TauTransition
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.TransitionNames
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.DataProvisioningHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageTo
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action

@Singleton
class EventExecutionGenerator {
	@Inject extension TransitionNames
	@Inject extension DataProvisioningHelper

	def compilePerformEventEngine(Robot robot) {
		val controllableEvents = robot.allTauTransitions + robot.allCommunicationTypeResets +
			robot.allCommunicationTypeTriggers + robot.allCommunicationTypeCancels
		val dataEvents = robot.allDataTransitions

		return '''
			«IF !dataEvents.empty»
				int nOfDataEvents = «dataEvents.size»;
				      «CifSynthesisTool.codePrefix»_Event_ data_events[«dataEvents.size»] = { «dataEvents.join(",")» };
				
				// Always execute data transitions that are possible
				shuffle_events(data_events, nOfDataEvents);
				
				for (int i = 0; i < nOfDataEvents; i++) {
					«CifSynthesisTool.codePrefix»_EnginePerformEvent(data_events[i]);
				}
			«ENDIF»
			
			«IF !controllableEvents.empty»
				int nOfControllableEvents = «controllableEvents.size»;
				      «CifSynthesisTool.codePrefix»_Event_ controllable_events[«controllableEvents.size»] = { «controllableEvents.join(",")» };
				
				shuffle_events(controllable_events, nOfControllableEvents);
				
				for (int i = 0; i < nOfControllableEvents; i++) {
					«CifSynthesisTool.codePrefix»_EnginePerformEvent(controllable_events[i]));
				}
			«ENDIF»
		'''
	}
	
	def compileSilentEventEngine(Robot robot) {
		val controllableEvents = robot.allTauTransitions + robot.allCommunicationTypeResets 
			+ robot.allCommunicationTypeCancels

		return '''
			«IF !controllableEvents.empty»
				int nOfControllableEvents = «controllableEvents.size»;
				      «CifSynthesisTool.codePrefix»_Event_ controllable_events[«controllableEvents.size»] = { «controllableEvents.join(",")» };
				
				shuffle_events(controllable_events, nOfControllableEvents);
				
				for (int i = 0; i < nOfControllableEvents; i++) {
					«CifSynthesisTool.codePrefix»_EnginePerformEvent(controllable_events[i]);
				}
			«ENDIF»
		'''
	}

	private def allTauTransitions(Robot robot) {
		return ModelHelper.findWithinRobot(robot, TauTransition).map[it.transitionName]
	}

	private def allCommunicationTypeTriggers(Robot robot) {
		return ModelHelper.findWithinRobot(robot, CommunicationType)
			.filter[!(it instanceof Message) || (it as Message).direction instanceof MessageTo]
			.map[it.triggerTransitionName]
	}

	private def allCommunicationTypeCancels(Robot robot) {
		return ModelHelper.findWithinRobot(robot, CommunicationType)
			.filter[it instanceof Action]
			.map[it.cancelTransitionName]
	}

	private def allCommunicationTypeResets(Robot robot) {
		return ModelHelper.findWithinRobot(robot, CommunicationType).filter[!(it instanceof Message)].map [
			it.resetTransitionName
		]
	}

	private def allDataTransitions(Robot robot) {
		return ModelHelper.findWithinRobot(robot, CommunicationType).flatMap[it.provideStatements(robot)].map [
			it.transitionName
		]
	}
}
