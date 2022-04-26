package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType

@Singleton
class ParameterNames {
	def sourceInputName(EnumDataType input)'''input'''
}