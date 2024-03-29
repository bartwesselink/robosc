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
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.expressions.ExpressionGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.IdentifierNamerInterface
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.DefaultIdentifierNamer
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config

abstract class AbstractCommunicationTypeGenerator<T extends CommunicationType> {
	@Inject extension MethodNames
	@Inject extension ObjectHelper
	@Inject extension EliminationChecker
	@Inject extension VariableNames
	@Inject EliminationHelper eliminationHelper
	@Inject EliminationChecker eliminationChecker
	@Inject DefaultIdentifierNamer defaultIdentifierNamer
	@Inject extension ExpressionGenerator

	abstract def CharSequence initializeField(T entity, Robot robot, Config config)
	abstract def CharSequence declareField(T entity, Robot robot, Config config)
	abstract def CharSequence functions(T entity, Robot robot, Config config)

	protected def prepareResult(CommunicationType entity, DataType type, Robot robot, String access)'''
	«IF type.simple»
	«IF eliminationChecker.communicationTypeInputRequiredInSynthesis(robot, entity)»
	«entity.inputName('response')» = «type.checkDataAccess(type.dataAccessProperty(access))»;
	«ENDIF»
	
	«FOR assignment : eliminationHelper.findAllCodeOnlyAssignments(robot, entity)»
	«assignment.assignmentVariable» = «assignment.value.compile(type.checkDataAccess(type.dataAccessProperty(access)))»;
	«ENDFOR»
	
	«ELSEIF !type.simple»
	«FOR pair : type.allProperties»
	«IF eliminationChecker.communicationTypeWithPropertyInputRequiredInSynthesis(robot, entity, pair.property)»
	«entity.inputName('response_' + pair.identifier)» = «type.checkDataAccess(access + '->' + pair.identifier)»;
	«ENDIF»
	
	«FOR assignment : eliminationHelper.findAllCodeOnlyAssignments(robot, entity, pair.property)»
	«assignment.assignmentVariable» = «assignment.value.compile(type.checkDataAccess(access + '->'))»;
	«ENDFOR»
	«ENDFOR»
	«ENDIF»
	'''
		
	protected def topicName(CommunicationType type) {
		return this.topicName(type, null)
	}
		
	protected def topicName(CommunicationType type, IdentifierNamerInterface namer) {
		if (namer !== null) {
			return namer.name(type)
		} else {
			return this.defaultIdentifierNamer.name(type)
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
