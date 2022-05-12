package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.NoneDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.EliminationChecker
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.ObjectHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.DataTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames

@Singleton
class InitializationGenerator {
	@Inject EliminationChecker eliminationChecker
	@Inject extension ObjectHelper
	@Inject extension DataTypeGenerator
	@Inject extension PlantNames
	
	def initializeEngineVariables(Robot robot)'''
	bool assigned = false;
	
	void «CifSynthesisTool.codePrefix»_AssignInputVariables() {
		if (assigned) return;
		
		«FOR communication : ModelHelper.findWithinRobot(robot, CommunicationType)»
		«communication.initializeInputs(robot)»
		«ENDFOR»
		
		assigned = true;
	}
	'''
	
	private def dispatch String initializeInputs(Message entity, Robot robot)'''
		«IF entity.direction instanceof MessageFrom»
		«entity.type.inputs(robot, entity, 'response')»
		«ENDIF»
	''' 
	
	private def dispatch String initializeInputs(Service entity, Robot robot)'''
		«entity.responseType.inputs(robot, entity, 'response')»
	''' 
	
	private def dispatch String initializeInputs(Action entity, Robot robot)'''
		«entity.feedbackType.inputs(robot, entity, 'feedback')»
		«entity.responseType.inputs(robot, entity, 'response')»
	''' 
	
	private def inputs(DataType dataType, Robot robot, CommunicationType communication, String prefix) {
		if (dataType instanceof NoneDataType) {
			return null
		} else if (dataType instanceof ComplexDataTypeReference) {
			val type = dataType.type

			if (type instanceof EnumDataType) {
				return dataType.basic(robot, communication, prefix)
			} else if (type instanceof ObjectDataType) {
				return '''
					«FOR pair : type.flattenProperties(prefix + '_').filter[eliminationChecker.communicationTypeWithPropertyInputRequiredInSynthesis(robot, communication, it.property)]»
						«communication.plantName»_i_«pair.identifier»_ = «pair.property.type.initialValue»;
					«ENDFOR»
				'''
			}
		} else if (dataType instanceof BasicDataType) {
			return dataType.basic(robot, communication, prefix)
		} else {
			return null
		}
	}

	private def basic(DataType dataType, Robot robot, CommunicationType communication, String prefix) {
		if (eliminationChecker.communicationTypeInputRequiredInSynthesis(robot, communication)) {
			return '''«communication.plantName»_i_«prefix»_ = «dataType.initialValue»;'''

		}

		return null
	}
}