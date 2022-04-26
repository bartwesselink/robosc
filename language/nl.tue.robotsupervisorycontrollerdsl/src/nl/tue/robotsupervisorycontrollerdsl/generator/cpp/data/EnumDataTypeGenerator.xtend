package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data

import javax.inject.Singleton
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.expressions.ExpressionGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumTransformationRule
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.EnumHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.MethodNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumValue

@Singleton
class EnumDataTypeGenerator {
	@Inject extension ExpressionGenerator
	@Inject extension DataTypeGenerator
	@Inject extension EnumHelper
	@Inject extension MethodNames
	
	def compile(EnumDataType ^enum)'''
	«enumTypeName» «^enum.convertMethod»(const «enum.parameterInputType» value) {
		«FOR rule : enum.rules.filter(EnumTransformationRule) SEPARATOR '\n} else '»if («rule.expression.compile») {
			return «rule.value.correspondingEngineType»;«ENDFOR»
		}
	
		return «enum.defaultRule.value.correspondingEngineType»;
	}
	'''

	def correspondingEngineType(EnumValue value) {
		return '''_«CifSynthesisTool.codePrefix»_«value.name»'''
	}
	
	def parameterInputType(EnumDataType ^enum) {
		val type = enum.type
		
		if (type instanceof ComplexDataTypeReference) {
			return '''«enum.typeSettings?.name»::SharedPtr'''
		} else if (type instanceof BasicDataType) {
			return type.compile
		}
	}
}