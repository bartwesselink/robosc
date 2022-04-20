package nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action

@Singleton
class PlantNames {
	def dispatch plantName(Component component)'''component_«component.name»'''
	def dispatch plantName(Service service)'''service_«service.name»'''
	def dispatch plantName(Message message)'''message_«message.name»'''
	def dispatch plantName(Action action)'''action_«action.name»'''
	def dataPlantName(CommunicationType communicationType)'''data_«communicationType.name»'''
}