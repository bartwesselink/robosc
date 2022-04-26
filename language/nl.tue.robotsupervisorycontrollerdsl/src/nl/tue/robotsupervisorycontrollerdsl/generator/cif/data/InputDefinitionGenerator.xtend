package nl.tue.robotsupervisorycontrollerdsl.generator.cif.data

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.ObjectHelper
import com.google.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.NoneDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.EliminationChecker
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType

@Singleton
class InputDefinitionGenerator {
	@Inject extension ObjectHelper
	@Inject extension DataTypeGenerator
	@Inject EliminationChecker eliminationChecker

	def inputs(DataType dataType, Robot robot, CommunicationType communication, String prefix) {
		if (dataType instanceof NoneDataType) {
			return null
		} else if (dataType instanceof ComplexDataTypeReference) {
			val type = dataType.type

			if (type instanceof EnumDataType) {
				return dataType.basic(robot, communication, prefix)
			} else if (type instanceof ObjectDataType) {
				return '''
					«FOR pair : type.flattenProperties(prefix).filter[eliminationChecker.communicationTypeWithPropertyInputRequiredInSynthesis(robot, communication, it.property)]»
						input «pair.property.type.compile» i_«pair.identifier»;
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
			return '''input «dataType.compile» i_«prefix»;'''

		}

		return null
	}
}
