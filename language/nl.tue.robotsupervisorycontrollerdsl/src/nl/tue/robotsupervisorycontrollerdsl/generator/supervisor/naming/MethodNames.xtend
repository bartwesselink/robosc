package nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.naming

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service

class MethodNames extends nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.MethodNames {
	def handleGoalSupervised(Action entity) '''handle_goal_«entity.name»_supervised'''
	def handleAcceptedSupervised(Action entity) '''handle_accepted_«entity.name»_supervised'''
	def handleCancelledSupervised(Action entity) '''handle_cancelled_«entity.name»_supervised'''
	def handleSupervised(Service entity) '''handle_«entity.name»_supervised'''
	def callbackMethodSupervised(Message entity) '''«entity.callbackMethod»_supervised'''
}