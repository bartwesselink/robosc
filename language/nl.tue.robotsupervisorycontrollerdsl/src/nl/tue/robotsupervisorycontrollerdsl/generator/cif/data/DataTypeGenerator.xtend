package nl.tue.robotsupervisorycontrollerdsl.generator.cif.data

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BooleanDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.StringDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DoubleDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.NoneDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.IntegerDataType

@Singleton
class DataTypeGenerator {
	def dispatch String compile(ComplexDataTypeReference reference)'''«reference.type.typeName»'''
	def dispatch String compile(BasicDataType type)'''«type.typeName»'''
	private def dispatch String typeName(BooleanDataType type)'''bool'''
	private def dispatch String typeName(StringDataType type)'''string'''
	private def dispatch String typeName(DoubleDataType type)'''real'''
	private def dispatch String typeName(IntegerDataType type)'''int[«IF type.from !== null»«type.from.value»«ENDIF»..«IF type.to !== null»«type.to.value»«ENDIF»]'''
	private def dispatch String typeName(NoneDataType type)''''''
	private def dispatch String typeName(EnumDataType type)'''«type.name»'''
}