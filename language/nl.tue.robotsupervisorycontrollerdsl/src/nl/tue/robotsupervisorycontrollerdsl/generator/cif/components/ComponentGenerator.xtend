package nl.tue.robotsupervisorycontrollerdsl.generator.cif.components

import nl.tue.robotsupervisorycontrollerdsl.generator.cif.automaton.AutomatonGenerator
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComponentBehaviour
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LocalComponent
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ImportedComponent
import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType

@Singleton
class ComponentGenerator {
	@Inject extension AutomatonGenerator
	@Inject extension CommunicationTypeGenerator
	@Inject extension PlantNames
	
	def compile(Component component, Robot robot)'''
	«component.behaviour?.compile(component, robot)»

	«FOR communication : component.communicationTypes»
	«communication.compile(robot)»
	«ENDFOR»
	'''
	
	def compile(ComponentBehaviour behaviour, Component parent, Robot robot)'''
	plant «parent.plantName»:
		«behaviour.automaton?.compile(robot)»
	end
	'''
	
	def ComponentBehaviour behaviour(Component component) {
		val componentType = component.type
		
    	if (componentType instanceof LocalComponent) {
    		return componentType.definitions.findFirst[it instanceof ComponentBehaviour] as ComponentBehaviour
    	} else if (componentType instanceof ImportedComponent) {
    		val definition = componentType.definition
    		
    		if (definition instanceof Component) {
    			return definition.behaviour
    		}
    	}
    }
	
	def Iterable<CommunicationType> communicationTypes(Component component) {
		val componentType = component.type
		
    	if (componentType instanceof LocalComponent) {
    		return componentType.definitions.filter(CommunicationType)
    	} else if (componentType instanceof ImportedComponent) {
    		val definition = componentType.definition
    		
    		if (definition instanceof Component) {
    			return definition.communicationTypes
    		}
    	}
    }
}