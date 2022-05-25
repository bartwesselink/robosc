package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom

@Singleton
class FieldNames {
	def dispatch fieldName(Message entity) '''«IF entity.direction instanceof MessageFrom»subscriber«ELSE»publisher«ENDIF»_client_«entity.name»'''
	def dispatch fieldName(Service entity) '''service_client_«entity.name»'''
	def dispatch fieldName(Action entity) '''action_client_«entity.name»'''
}

