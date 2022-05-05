package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.info

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.components.ComponentGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractPlatformTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.VariableNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AutomatonVariable

@Singleton
class InfoUtilitiesGenerator {
	@Inject extension ComponentGenerator
	@Inject extension PlantNames
	@Inject extension VariableNames

	val fields = newArrayList("activated_messages", "activated_services", "activated_actions",
		"received_response_messages", "received_response_services", "received_response_actions",
		"received_feedback_actions")

	def compileInfoFunction(Robot robot, AbstractPlatformTypeGenerator platformType) '''
		void emit_current_state() {
			std::stringstream output;
			
			output << "{";
			
			output << "\"components\": {" << "";
			«FOR component : robot.componentsWithBehaviour»
				output << "\"«component.name»\": {";
				output << "\"state\": \"" << enum_names[«component.plantName»] << "\",";
				output << "\"variables\": {";
				
				«FOR variable : component.componentVariables»
				output << "\"«variable.name»\": \"" << «variable.variableName(robot)» << "\"«IF !variable.lastVariable(component)»,«ENDIF»";				
				«ENDFOR»
				
				output << "}";
				output << "}«IF !component.lastBehaviourComponent(robot)»,«ENDIF»";
			«ENDFOR»
			output << "},";
			«FOR field : fields»
			output << "\"«field»\": " << serialize_json_vector(«field») << "«IF !field.lastField»,«ENDIF»";
			«ENDFOR»
			output << "}";
			
			«platformType.informationPublisher»

			«FOR field : fields»
			«field».clear();
			«ENDFOR»
		}
	'''

	def compileActivationFields(Robot robot) '''
		«FOR field : fields»
		std::vector<std::string> «field»;
		«ENDFOR»
	'''

	def componentsWithBehaviour(Robot robot) {
		return robot.definitions.filter(Component).filter[it.behaviour !== null]
				.toList
	}

	def componentVariables(Component component) {
		val behaviour = component.behaviour
		
		if (behaviour !== null) {
			return behaviour
				.automaton
				.definitions
				.filter(AutomatonVariable)
				.map[it.variable]
				.toList
		}
		
		return newArrayList
	}
	
	def lastField(String field) {
		val index = fields.indexOf(field)
		
		return index + 1 == fields.size
	}
	
	def lastBehaviourComponent(Component component, Robot robot) {
		val index = robot.componentsWithBehaviour.indexOf(component)
		
		return index + 1 == robot.componentsWithBehaviour.size
	}
	
	def lastVariable(Variable variable, Component component) {
		val index = component.componentVariables.indexOf(variable)
		
		return index + 1 == component.componentVariables.size
	}
}
