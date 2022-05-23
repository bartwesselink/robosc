package nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.TauTransitionNamer
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TauTransition
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import com.google.common.util.concurrent.Service
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.ProvideStatementNamer
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.FeedbackResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResponseResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RequestResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ErrorResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CancelResultType

@Singleton
class TransitionNames {
	@Inject TauTransitionNamer tauTransitionNamer
	@Inject ProvideStatementNamer provideStatementNamer

	def transitionName(TauTransition tauTransition)'''c_«tauTransitionNamer.getName(tauTransition)»'''
	def transitionName(ProvideStatement statement)'''c_«provideStatementNamer.getName(statement)»'''
	def transitionName(Message message) {
		if (message.direction instanceof MessageFrom) {
			return 'u_response'
		} else {
			return 'c_trigger'
		}
	}

	def dispatch triggerTransitionName(Message message)'''c_trigger'''
	def dispatch triggerTransitionName(Service service)'''c_trigger'''
	def dispatch triggerTransitionName(Action action)'''c_trigger'''

	def dispatch cancelTransitionName(Message message)''''''
	def dispatch cancelTransitionName(Service service)''''''
	def dispatch cancelTransitionName(Action action)'''c_cancel'''

	def dispatch resetTransitionName(Message message) {
		return null
	}
	def dispatch resetTransitionName(Service service)'''c_reset'''
	def dispatch resetTransitionName(Action action)'''c_reset'''
	
	def dispatch uncontrollableTransitionName(RequestResultType type)'''«triggerTransitionName»'''
	def dispatch uncontrollableTransitionName(FeedbackResultType type)'''«feedbackTransitionName»'''
	def dispatch uncontrollableTransitionName(CancelResultType type)'''«cancelTransitionName»'''
	def dispatch uncontrollableTransitionName(ResponseResultType type)'''«responseTransitionName»'''
	def dispatch uncontrollableTransitionName(ErrorResultType type)'''«errorTransitionName»'''
	
	def triggerTransitionName()'''c_trigger'''
	def cancelTransitionName()'''c_cancel'''
	def feedbackTransitionName()'''u_feedback'''
	def responseTransitionName()'''u_response'''
	def errorTransitionName()'''u_error'''
}

