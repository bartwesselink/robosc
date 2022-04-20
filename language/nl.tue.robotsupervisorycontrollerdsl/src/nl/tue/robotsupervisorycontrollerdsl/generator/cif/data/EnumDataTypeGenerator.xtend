package nl.tue.robotsupervisorycontrollerdsl.generator.cif.data

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import java.util.Set
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumValue

@Singleton
class EnumDataTypeGenerator {
	def compile(EnumDataType ^enum) {
		val Set<EnumValue> values = newHashSet()
    	values.addAll(enum.rules.map[it.value])
    	
    	return '''enum «enum.name» = «values.map[it.name].join(',')»;'''
	}
}