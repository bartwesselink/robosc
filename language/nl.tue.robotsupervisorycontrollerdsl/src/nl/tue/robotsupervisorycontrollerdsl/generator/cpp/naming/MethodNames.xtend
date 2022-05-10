package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action

@Singleton
class MethodNames {
	def callbackMethod(Message entity) '''callback_message_«entity.name»'''
	def callMethod(Message entity) '''call_message_«entity.name»'''
	def callMethod(Service entity) '''call_service_«entity.name»'''
	def responseMethod(Service entity) '''response_service_«entity.name»'''
	def callMethod(Action entity) '''call_action_«entity.name»'''
	def cancelMethod(Action entity) '''cancel_action_«entity.name»'''
	def responseMethod(Action entity) '''response_action_«entity.name»'''
	def feedbackMethod(Action entity) '''feedback_action_«entity.name»'''
	def convertMethod(EnumDataType entity) '''convert_enum_«entity.name»'''
}

