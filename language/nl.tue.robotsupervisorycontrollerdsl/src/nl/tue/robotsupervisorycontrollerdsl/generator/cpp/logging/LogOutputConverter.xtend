package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.logging

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.DefaultIdentifierNamer
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action

@Singleton
class LogOutputConverter {
	@Inject DefaultIdentifierNamer namer
	
	dispatch def responseOutput(Message entity)'''Response from message «entity.name» (identifier «namer.name(entity)») '''
	dispatch def responseOutput(Service entity)'''Response from service «entity.name» (identifier «namer.name(entity)») '''
	dispatch def responseOutput(Action entity)'''Response from action «entity.name» (identifier «namer.name(entity)») '''
	
	dispatch def requestOutput(Message entity)'''Request to message «entity.name» (identifier «namer.name(entity)») '''
	dispatch def requestOutput(Service entity)'''Request to service «entity.name» (identifier «namer.name(entity)») '''
	dispatch def requestOutput(Action entity)'''Request to action «entity.name» (identifier «namer.name(entity)») '''

	def feedbackOutput(Action entity)'''Feedback from action «entity.name» (identifier «namer.name(entity)») '''
	def cancelOutput(Action entity)'''Cancelling action «entity.name» (identifier «namer.name(entity)») '''
}