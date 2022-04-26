package nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.data.DataTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CustomTypeSettings
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import com.google.common.base.CaseFormat

@Singleton
class PlatformTypeGenerator {
	@Inject extension DataTypeGenerator
	
	def String messageType(DataType data, CustomTypeSettings settings) {
		if (data instanceof ComplexDataTypeReference) {
			val referenced = data.type
			
			if (referenced instanceof EnumDataType) {
				return referenced.type.messageType(referenced.typeSettings)
			} else {
				return '''«settings.package»::msg::«settings.name»'''
			}
		} else if (data instanceof BasicDataType) {
			return data.compile
		}
	}
	
	def actionType(CustomTypeSettings settings) {
		return  '''«settings.package»::action::«settings.name»'''
	}
	
	def serviceType(CustomTypeSettings settings) {
		return  '''«settings.package»::srv::«settings.name»'''
	}
	
	def String messageImport(CustomTypeSettings settings) {
		return '''«settings.package»/msg/«settings.name.snakeCase»'''
	}
	
	def actionImport(CustomTypeSettings settings) {
		return  '''«settings.package»/action/«settings.name.snakeCase»'''
	}
	
	def serviceImport(CustomTypeSettings settings) {
		return  '''«settings.package»/srv/«settings.name.snakeCase»'''
	}
	
	def snakeCase(String input) {
		return CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, input)
	}
}