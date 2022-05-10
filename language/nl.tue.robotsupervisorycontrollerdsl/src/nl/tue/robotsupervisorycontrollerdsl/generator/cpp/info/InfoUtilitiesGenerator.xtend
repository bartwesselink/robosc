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
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RequestResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResponseResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.FeedbackResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ErrorResultType

@Singleton
class InfoUtilitiesGenerator {
  	@Inject Serializer serializer
	@Inject extension ComponentGenerator
	@Inject extension PlantNames
	@Inject extension VariableNames
	@Inject extension TransitionNames

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
			output << "\"transitions\": " << serialize_json_vector(taken_transitions) << ",";
			output << "\"definition\": " << "«robot.serialize.replace('"', '\\"')»";
			output << "}";
			
			«platformType.informationPublisher»

			taken_transitions.clear();
		}
	'''

	def compileActivationFields(Robot robot) '''
		std::vector<std::string> taken_transitions;
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
		val result = new JsonObject()
		
		val variables = new JsonArray()
		
		for (item : input.automaton.definitions.filter(AutomatonVariable)) {
			variables.add(item.variable.name)
		}
		
		val states = new JsonArray()
			
		for (state : input.automaton.definitions.filter(State)) {
			val object = new JsonObject()
			
			object.addProperty("name", state.name)
			object.addProperty("initial", state.initial)
			object.add("transitions", state.serializeTransitions(input.automaton))

			states.add(object)
		}
		
		result.add("variables", variables)
		result.add("states", states)
		
		return result
	}
	
	def serializeTransitions(State input, Automaton automaton) {
		val transitions = new JsonArray()
		val all = input.transitions + automaton.definitions.filter(Transition)
		
		for (transition : all) {
			val object = new JsonObject()

			object.addProperty("next", transition.stateChange?.state?.name)
			object.addProperty("id", transition.id.toString)

			if (transition instanceof ResultTransition) {
				object.addProperty("type", serializer.serialize(transition.resultType).trim())
				object.addProperty("communication", transition.communicationType.name)
			} else if (transition instanceof TauTransition) {
				object.addProperty("type", "tau")
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
	
	private def dispatch id(ResultTransition transition) {
		if (transition.resultType instanceof RequestResultType) {
			return transition.communicationType.triggerTransitionName
		} else if (transition.resultType instanceof ResponseResultType) {
			return transition.communicationType.responseTransitionName
		} else if (transition.resultType instanceof FeedbackResultType) {
			return transition.communicationType.feedbackTransitionName
		} else if (transition.resultType instanceof ErrorResultType) {
			return transition.communicationType.errorTransitionName
		}
		
		return null
	}
	
	private def dispatch id(TauTransition transition) {
		return transition.transitionName
	}
}
