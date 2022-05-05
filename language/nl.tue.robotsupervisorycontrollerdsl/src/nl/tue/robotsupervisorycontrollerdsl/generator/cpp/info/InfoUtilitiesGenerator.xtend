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
import com.google.gson.JsonObject
import com.google.gson.JsonArray
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComponentBehaviour
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Transition
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Automaton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResultTransition
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TauTransition
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import com.google.gson.JsonElement
import com.google.common.util.concurrent.Service
import org.eclipse.xtext.serializer.impl.Serializer
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message

@Singleton
class InfoUtilitiesGenerator {
  	@Inject Serializer serializer
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
			
			output << "\"current\": {" << "";
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
			output << "\"«field»\": " << serialize_json_vector(«field») << ",";
			«ENDFOR»
			output << "\"definition\": " << "«robot.serialize.replace('"', '\\"')»";
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
		return robot.components.filter[it.behaviour !== null]
				.toList
	}

	def components(Robot robot) {
		return robot.definitions.filter(Component)
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
	
	def serialize(Robot robot) {
		val output = new JsonObject()

		output.addProperty("name", robot.name)
		
		val	components = new JsonArray()
		for (definition : robot.components) {
			components.add(definition.serialize(robot))
		}
		
		output.add("components", components)
		
		return output.toString
	}
	
	def serialize(Component input, Robot robot) {
		val component = new JsonObject()
		
		component.addProperty("name", input.name)
		component.add("messages", input.communicationTypes.filter(Message).serializeCommunicationTypes as JsonElement)
		component.add("services", input.communicationTypes.filter(Service).serializeCommunicationTypes as JsonElement)
		component.add("actions", input.communicationTypes.filter(Action).serializeCommunicationTypes as JsonElement)

		val behaviour = input.behaviour
		if (behaviour !== null) {
			component.add("behaviour", behaviour.serialize)
		}
		
		return component
	}
	
	def serialize(ComponentBehaviour input) {
		val states = new JsonArray()
			
		for (state : input.automaton.definitions.filter(State)) {
			val object = new JsonObject()
			
			object.addProperty("name", state.name)
			object.addProperty("initial", state.initial)
			object.add("transitions", state.serializeTransitions(input.automaton))

			states.add(object)
		}
		
		return states
	}
	
	def serializeTransitions(State input, Automaton automaton) {
		val transitions = new JsonArray()
		val all = input.transitions + automaton.definitions.filter(Transition)
		
		for (transition : all) {
			val object = new JsonObject()

			object.addProperty("next", transition.stateChange?.state?.name)
			
			if (transition instanceof ResultTransition) {
				object.addProperty("type", serializer.serialize(transition.resultType))
				object.addProperty("communication", transition.communicationType.name)

				if (transition.assignment?.assignment !== null) {
					object.addProperty("assignment", serializer.serialize(transition.assignment.assignment))
				}
			} else if (transition instanceof TauTransition) {
				object.addProperty("type", "tau")
				
				if (transition.guard?.expression !== null) {
					object.addProperty("guard", serializer.serialize(transition.guard.expression))
				}
			}
			
			transitions.add(object)
		}	

		return transitions
	}
	
	def serializeCommunicationTypes(Iterable<?> input) {
		val output = new JsonArray()
		
		for (communication : input) {
			output.add((communication as CommunicationType).name)
		}
		
		return output
	}
}
