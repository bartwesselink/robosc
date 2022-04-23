package nl.tue.robotsupervisorycontrollerdsl.generator.cif.automaton

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Automaton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TauTransition
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TransitionStateChange
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TransitionGuard
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResultTransition
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.expressions.ExpressionGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.data.DataTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.VariableNames
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.EliminationChecker
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AutomatonVariable
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TransitionAssignment
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResponseResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.FeedbackResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Assignment

@Singleton
class AutomatonGenerator {
	@Inject extension TransitionNames
	@Inject extension VariableNames
	@Inject extension PlantNames
	@Inject extension ExpressionGenerator
	@Inject extension DataTypeGenerator
	@Inject EliminationChecker eliminationChecker

	def compile(Automaton automaton, Robot robot) '''
		«IF !automaton.allTauTransitions.isEmpty»
			controllable «FOR event : automaton.allTauTransitions SEPARATOR ', '»«event.transitionName»«ENDFOR»;
		«ENDIF»
		
		«FOR variable : automaton.definitions.filter(AutomatonVariable).filter[eliminationChecker.variableRequiredInSynthesis(robot, it.variable)]»
			«variable.variable.compile»
		«ENDFOR»
		
		«FOR state : automaton.states»«state.compile(robot, automaton)»«ENDFOR»
	'''

	private def allowUncontrollableEvents(Robot robot, Automaton automaton) '''
		// allow uncontrollable events to occur, if not specified
		«FOR type : ModelHelper.findWithinRobot(robot, CommunicationType)»
			«type.allowUncontrollableEventsEdge(automaton)»
		«ENDFOR»
		// end allow uncontrollable events to occur
	'''

	private def dispatch String allowUncontrollableEventsEdge(Message message, Automaton automaton) {
		if (message.direction instanceof MessageFrom) {
			if (automaton.allResultTransitions(message).empty) {
				return '''
					edge «message.plantName».«message.transitionName»;
				'''
			}
		}
	}

	private def dispatch String allowUncontrollableEventsEdge(Service service, Automaton automaton) {
		if (automaton.allResultTransitions(service).filter[it.resultType instanceof ResponseResultType].empty) {
			return '''
				edge «service.plantName».«responseTransitionName»;
			'''
		}
	}

	private def dispatch String allowUncontrollableEventsEdge(Action action, Automaton automaton)'''
		«IF automaton.allResultTransitions(action).filter[it.resultType instanceof ResponseResultType].empty»
		edge «action.plantName».«responseTransitionName»;
		«ENDIF»
		
		«IF automaton.allResultTransitions(action).filter[it.resultType instanceof FeedbackResultType].empty»
		edge «action.plantName».«feedbackTransitionName»;
		«ENDIF»
	'''

	private def allResultTransitions(Automaton automaton, CommunicationType type) {
		return (
			automaton.definitions.filter(ResultTransition) +
			automaton.definitions.filter(State).flatMap[it.transitions].filter(ResultTransition)
		).filter[it.communicationType == type]
	}

	private def compile(State state, Robot robot, Automaton automaton) '''
		location «state.name»:
			«IF state.initial»initial;«ENDIF» marked;
					
			«robot.allowUncontrollableEvents(automaton)»
			
			«FOR transition : state.transitions»
				«transition.edge(robot)»
			«ENDFOR»
	'''

	private def dispatch edge(
		TauTransition transition, Robot robot) '''edge «transition.transitionName»«transition.guard?.compile»«transition.stateChange?.compile»;'''

	private def dispatch edge(ResultTransition transition, Robot robot) {
		return '''edge «transition.communicationType.plantName».«transition.resultType.uncontrollableTransitionName» «transition.assignment?.compile(robot)» «transition.stateChange?.compile»;'''
	}

	private def allTauTransitions(Automaton automaton) {
		val globalTauTransitions = automaton.definitions.filter(TauTransition)
		val localTauTransitions = automaton.definitions.filter(State).flatMap[it.transitions.filter(TauTransition)]

		return (globalTauTransitions + localTauTransitions)
	}

	private def compile(TransitionStateChange stateChange) ''' goto «stateChange.state.name»'''

	private def compile(TransitionGuard guard) ''' when «guard.expression.compile»'''

	private def compile(TransitionAssignment transitionAssignment, Robot robot) {
		val expression = transitionAssignment.assignment as Assignment
		
		if (this.eliminationChecker.assignmentRequiredInSynthesis(robot, expression)) {
			return ''' do «transitionAssignment.assignment.compile»'''
		}
	}

	private def compile(
		Variable variable) '''disc «variable.type.compile» «variable.variableName»«IF variable.initial !== null» = «variable.initial.compile»«ENDIF»;'''

	private def states(Automaton automaton) {
		return automaton.definitions.filter(State)
	}
}
