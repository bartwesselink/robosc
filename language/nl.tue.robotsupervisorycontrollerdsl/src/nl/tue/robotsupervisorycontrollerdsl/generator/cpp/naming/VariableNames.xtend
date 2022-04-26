package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.EliminationChecker
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot

@Singleton
class VariableNames {
	@Inject PlantNames cifPlantNames
	@Inject nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.VariableNames cifVariableNames
	@Inject EliminationChecker eliminationChecker
	
	def inputName(CommunicationType communicationType, String prefix) '''«cifPlantNames.plantName(communicationType)»_i_«prefix»_'''
	def codeOnlyVariableName(Variable variable) {
		val component = ModelHelper.findParentOfType(variable, Component)
		
		return '''code_«component.name»_«variable.name»'''
	}
	
	
	def variableName(Variable variable, Robot robot) {
		if (eliminationChecker.variableRequiredInSynthesis(robot, variable)) {
			val component = ModelHelper.findParentOfType(variable, Component)
			
			return '''«cifPlantNames.plantName(component)»_«cifVariableNames.variableName(variable)»_'''
		} else {
			return variable.codeOnlyVariableName
		}
	}
}