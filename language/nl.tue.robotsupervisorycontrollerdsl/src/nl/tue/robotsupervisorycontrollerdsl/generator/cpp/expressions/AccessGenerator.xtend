package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.expressions

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LiteralValue
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumValue
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import javax.inject.Inject
import java.util.List
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.VariableNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.ParameterNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.EnumDataTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AccessType
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.AccessHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.PlantNames

@Singleton
class AccessGenerator {
	@Inject extension VariableNames
	@Inject extension ParameterNames
	@Inject extension PlantNames
	@Inject extension EnumDataTypeGenerator
	@Inject extension AccessHelper

	def compilePath(Access access) {
		return compilePath(access, null)
	}

	def compilePath(Access access, String literalValue) {
		if (access.value !== null) return access.value.compileFirst(access, literalValue)
		
		return access.firstItem.compileFirst(access, literalValue)
	}

	def dispatch compileFirst(LiteralValue entity, Access access, String literalValue) {
		val enumDataType = ModelHelper.findParentOfType(entity, EnumDataType)

		if (enumDataType !== null) {
			return enumDataType.sourceInputName + access.types.createAccessPath
		} else {
			return literalValue + access.types.createAccessPath
		}
	}

	def dispatch compileFirst(Component entity, Access access, String literalValue) {
		if(access.accessItems.size != 2) throw new UnsupportedOperationException("States cannot be accessed.")
		
		val secondItem = access.accessItems.get(1)
		
		if (secondItem instanceof State) {
			return '''(«entity.plantName» == _«CifSynthesisTool.codePrefix»_«secondItem.name»)''';
		} else if (secondItem instanceof Variable) {
			val robot = ModelHelper.findParentOfType(secondItem, Robot)
			
			return secondItem.variableName(robot);
		}
	}

	def dispatch compileFirst(Variable entity, Access access, String literalValue) {
		if(access.accessItems.size > 1) throw new UnsupportedOperationException("Variables cannot be accessed.")

		val robot = ModelHelper.findParentOfType(entity, Robot)

		return entity.variableName(robot)
	}

	def dispatch compileFirst(EnumValue entity, Access access, String literalValue) {
		if(access.accessItems.size > 1) throw new UnsupportedOperationException("Enums cannot be accessed.")

		return entity.correspondingEngineType
	}

	private def createAccessPath(List<AccessType> types) {
		if (types.empty) return ''
		
		var result = ''
		var first = true
		
		for (type : types) {
			if (type.item !== null && type.item instanceof ObjectProperty) {
				result += (!first ? '.' : '') + (type.item as ObjectProperty).name
			} else if (type.item === null) {
				result += '[' + type.index + ']'
			}
			
			first = false
		}

		return result
	}
}
