package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BooleanDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.StringDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DoubleDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.NoneDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.IntegerDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.EnumHelper
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractPlatformTypeGenerator

@Singleton
class DataTypeGenerator {
	@Inject extension EnumHelper	

	def String compile(BasicDataType type, AbstractPlatformTypeGenerator generator)'''«generator.platformType(type)»'''
	def String enumTypeName()'''controllerEnum'''


	def dispatch String primitiveType(BooleanDataType type)'''bool'''
	def dispatch String primitiveType(StringDataType type)'''string'''
	def dispatch String primitiveType(DoubleDataType type)'''double'''
	def dispatch String primitiveType(IntegerDataType type)'''int'''
	def dispatch String primitiveType(NoneDataType type)'''void'''
	
	
	def String initialValue(DataType type)'''«type.defaultValue»'''
	private def dispatch String defaultValue(BooleanDataType type)'''false'''
	private def dispatch String defaultValue(StringDataType type)'''""'''
	private def dispatch String defaultValue(DoubleDataType type)'''0.0'''
	private def dispatch String defaultValue(IntegerDataType type)'''0'''
	private def dispatch String defaultValue(NoneDataType type)''''''
	private def dispatch String defaultValue(ComplexDataTypeReference reference) {
		val type = reference.type
		
		if (type instanceof EnumDataType) {
			return '''_«CifSynthesisTool.codePrefix»_«type.defaultRule.value.name»'''
		}
	}
}