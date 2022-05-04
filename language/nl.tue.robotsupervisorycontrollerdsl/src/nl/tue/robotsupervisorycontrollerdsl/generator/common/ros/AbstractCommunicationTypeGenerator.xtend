package nl.tue.robotsupervisorycontrollerdsl.generator.common.ros

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.MethodNames
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.ObjectHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Assignment
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.EliminationChecker
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.VariableNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.EliminationHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Expression
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.expressions.ExpressionGenerator

abstract class AbstractCommunicationTypeGenerator<T extends CommunicationType> {
	@Inject extension MethodNames
	@Inject extension ObjectHelper
	@Inject extension EliminationChecker
	@Inject extension VariableNames
	@Inject EliminationHelper eliminationHelper
	@Inject EliminationChecker eliminationChecker
	@Inject extension ExpressionGenerator
	
	abstract def CharSequence initializeField(T entity, Robot robot)
	abstract def CharSequence declareField(T entity, Robot robot)
	abstract def CharSequence functions(T entity, Robot robot)

	protected def prepareResult(CommunicationType entity, DataType type, Robot robot, String access)'''
	«IF type.simple»
	«IF eliminationChecker.communicationTypeInputRequiredInSynthesis(robot, entity)»
	«entity.inputName('response')» = «type.checkDataAccess(type.dataAccessProperty(access))»;
	«ENDIF»
	
	«FOR assignment : eliminationHelper.findAllCodeOnlyAssignments(robot, entity)»
	«assignment.assignmentVariable» = «(assignment.value as Expression).compile(type.checkDataAccess(type.dataAccessProperty(access)))»;
	«ENDFOR»
	
	«ELSEIF !type.simple»
	«FOR pair : type.allProperties»
	«IF eliminationChecker.communicationTypeWithPropertyInputRequiredInSynthesis(robot, entity, pair.property)»
	«entity.inputName('response_' + pair.identifier)» = «type.checkDataAccess(access + '->' + pair.identifier.replace('_', '.'))»;
	«ENDIF»
	
	«FOR assignment : eliminationHelper.findAllCodeOnlyAssignments(robot, entity, pair.property)»
	«assignment.assignmentVariable» = «(assignment.value as Expression).compile(type.checkDataAccess(access + '->' + pair.identifier.replace('_', '.')))»;
	«ENDFOR»
	«ENDFOR»
	«ENDIF»
	'''
		
	protected def topicName(CommunicationType type) {
		if (type.identifier === null) {
			return type.name
		} else {
			return type.identifier
		}
	}
	
	protected def simple(DataType data) {
		return data instanceof BasicDataType || (
			data instanceof ComplexDataTypeReference
			&& (data as ComplexDataTypeReference).type instanceof EnumDataType
		)
	}
	
	protected def allProperties(DataType type) {
		if (type.simple) return newArrayList
		
		val complexReference = type as ComplexDataTypeReference
		val object = complexReference.type as ObjectDataType
		
		return object.flattenProperties('')
	}
	
	protected def checkDataAccess(DataType dataType, String access) {
		if (!dataType.simple) return access
		
		if (dataType instanceof ComplexDataTypeReference) {
			val enum = dataType.type as EnumDataType
			
			return '''«enum.convertMethod»(«access»)'''
		} else {
			return access
		}
	}
	
	protected def assignmentVariable(Assignment assignment) {
		return assignment.lastVariableOfAssignment.codeOnlyVariableName()
	}
	
	private def dataAccessProperty(DataType dataType, String access) {
		if (!dataType.simple) return access
		
		if (dataType instanceof ComplexDataTypeReference) {
			val enum = dataType.type as EnumDataType
			
			if (enum.type instanceof ComplexDataTypeReference) {
				return access
			}
		}
		
		return access + '->data'
	}
}
