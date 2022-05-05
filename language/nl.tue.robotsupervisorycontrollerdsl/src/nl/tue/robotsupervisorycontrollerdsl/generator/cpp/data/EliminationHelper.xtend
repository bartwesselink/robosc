package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.EliminationChecker
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResultTransition
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Assignment
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty
import java.util.stream.Collectors
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.VariableNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.AccessHelper

@Singleton
class EliminationHelper {
	@Inject EliminationChecker eliminationChecker
	@Inject extension VariableNames
	@Inject extension DataTypeGenerator
	@Inject extension AccessHelper

	def findAllCodeOnlyAssignments(Robot robot, CommunicationType type) {
		return ModelHelper.findWithinRobot(robot, ResultTransition)
			.filter[it.communicationType == type]
			.filter[it.assignment !== null]
			.map[it.assignment.assignment as Assignment]
			.filter[!eliminationChecker.assignmentRequiredInSynthesis(robot, it)]
			.flatMap[eliminationChecker.findLiteralValues(it.value).collect(Collectors.toList)]
			.map[ModelHelper.findParentOfType(it, Access)]
			
			// Ensure that we are not accessing any properties
			.filter[it.accessItems.empty]
			.map[ModelHelper.findParentOfType(it, Assignment)]
	}
	
	def findAllCodeOnlyAssignments(Robot robot, CommunicationType type, ObjectProperty property) {
		return ModelHelper.findWithinRobot(robot, ResultTransition)
			.filter[it.communicationType == type]
			.filter[it.assignment !== null]
			.map[it.assignment.assignment as Assignment]
			.filter[!eliminationChecker.assignmentRequiredInSynthesis(robot, it)]
			.flatMap[eliminationChecker.findLiteralValues(it.value).collect(Collectors.toList)]
			.map[ModelHelper.findParentOfType(it, Access)]
			
			// Ensure that we are not accessing any properties
			.filter[!it.accessItems.empty]
			.filter[eliminationChecker.accessPropertyIsSame(it, property)]
			
			.map[ModelHelper.findParentOfType(it, Assignment)]
	}
	
	def compileCodeOnlyVariables(Robot robot) {
		val components = ModelHelper.findWithinRobot(robot, Component)
		
		return '''
		«FOR component : components»
		«FOR variable : ModelHelper.findChildren(component, Variable)»
		«variable.type.typeName» «variable.codeOnlyVariableName» = «variable.type.initialValue»;
		«ENDFOR»
		«ENDFOR»
		'''		
	}
	
	private def typeName(DataType type) {
		if (type instanceof BasicDataType) {
			return type.primitiveType
		} else if (type instanceof ComplexDataTypeReference) {
			val referenced = type.type
			
			if (referenced instanceof EnumDataType) {
				return enumTypeName
			}
		}
	}
}
