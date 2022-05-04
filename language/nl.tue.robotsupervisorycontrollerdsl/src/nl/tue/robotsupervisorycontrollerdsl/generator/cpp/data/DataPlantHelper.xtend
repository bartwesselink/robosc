package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data

import javax.inject.Singleton
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.ObjectHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectValue
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.expressions.ExpressionGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Expression
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.StateNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.NoneDataType

@Singleton
class DataPlantHelper {
	@Inject extension ObjectHelper
	@Inject extension ExpressionGenerator
	@Inject extension PlantNames
	@Inject extension StateNames

	def compileDataStates(CommunicationType communication, DataType type, String dataVariable, Robot robot, Boolean pointer) {
		if (type.object) {
			return compileObjectDataState(communication, (type as ComplexDataTypeReference).type as ObjectDataType, dataVariable, robot, pointer)
		} else if (!(type instanceof NoneDataType)) {
			return compileSimpleDataState(communication, type, dataVariable, robot, pointer)
		}
	}
	
	private def compileObjectDataState(CommunicationType communication, ObjectDataType object, String dataVariable, Robot robot, Boolean pointer)'''
		«FOR pair : object.flattenProperties('')»
		«FOR statement : communication.allProvideStatementsWithData(robot) SEPARATOR '\n} else '»
		if («communication.dataPlantName» == «statement.dataLocationName») {
			«IF statement.objectValue.valueForProperty(pair.property, pair.identifier) !== null»
			«dataVariable»«pointer.accessMethod»«pair.identifier.replace('_', '.')» = «statement.objectValue.valueForProperty(pair.property, pair.identifier)?.compile»;
			«ENDIF»
		«ENDFOR»
		«IF !communication.allProvideStatementsWithData(robot).empty»}«ENDIF»
		«ENDFOR»
	'''
	
	private def compileSimpleDataState(CommunicationType communication, DataType object, String dataVariable, Robot robot, Boolean pointer)'''
		«FOR statement : communication.allProvideStatementsWithData(robot) SEPARATOR '\n} else '»
		if («communication.dataPlantName» == «statement.dataLocationName») {
			«dataVariable»«pointer.accessMethod»data = «(statement.data as Expression).compile»;
		«ENDFOR»
		«IF !communication.allProvideStatementsWithData(robot).empty»}«ENDIF»
	'''
	
	private def accessMethod(Boolean pointer) {
		if (pointer) {
			return '->'
		} else {
			return '.'
		}
	}
	
	private def Expression valueForProperty(ObjectValue value, ObjectProperty property, String path) {
		val parts = path.split('_')
    	var currentObject = value;
    	var i = 0
    	    	
    	for (part : parts) {
    		val last = (i + 1) == parts.length
    		val objectProperty = currentObject.properties.findFirst[it.property.name.equalsIgnoreCase(part)]
    		
    		if (objectProperty !== null) {
    			val newValue = objectProperty.value
    			
    			if (newValue instanceof ObjectValue) {
    				if (last) return null
    				
    				currentObject = newValue
    			} else {
    				if (!last) return null
    				
    				return objectProperty.value as Expression
    			}
    		} else {
    			return null
    		}
    		
    		i++
    	}
	}
	
	private def objectValue(ProvideStatement statement) {
		return statement.data.data as ObjectValue
	}
	
	private def allProvideStatementsWithData(CommunicationType type, Robot robot) {
		return ModelHelper.findWithinRobot(robot, ProvideStatement)
			.filter[it.communicationType == type]
			.filter[it.data !== null && it.data.data !== null]
	}	
	
	private def object(DataType data) {
		return data instanceof ComplexDataTypeReference && (data as ComplexDataTypeReference).type instanceof ObjectDataType
	}
}