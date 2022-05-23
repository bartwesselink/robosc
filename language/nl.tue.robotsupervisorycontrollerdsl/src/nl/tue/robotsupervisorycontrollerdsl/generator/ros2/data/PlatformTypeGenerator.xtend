package nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Interface
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractPlatformTypeGenerator
import com.google.common.base.CaseFormat
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.StringDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BooleanDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DoubleDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.IntegerDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.NoneDataType

@Singleton
class PlatformTypeGenerator extends AbstractPlatformTypeGenerator {	
	override String messageType(DataType data, Interface  ^interface) {
		if (data instanceof ComplexDataTypeReference) {
			val referenced = data.type
			
			if (referenced instanceof EnumDataType) {
				val enumDataType = referenced.type

				if (enumDataType instanceof BasicDataType) {
					return enumDataType.platformType
				}
	
				return '''«^interface.interfacePackage»::msg::«^interface.interfaceName»'''
			} else {
				return '''«^interface.interfacePackage»::msg::«^interface.interfaceName»'''
			}
		} else if (data instanceof BasicDataType) {
			return data.platformType
		}
	}
	
	override actionType(Interface ^interface) {
		return  '''«^interface.interfacePackage»::action::« ^interface.interfaceName»'''
	}
	
	override serviceType(Interface ^interface) {
		return  '''«^interface.interfacePackage»::srv::«^interface.interfaceName»'''
	}
	
	override String messageImport(Interface ^interface) {
		return '''«^interface.interfacePackage»/msg/«^interface.interfaceName.snakeCase»'''
	}
	
	override actionImport(Interface ^interface) {
		return  '''«^interface.interfacePackage»/action/«^interface.interfaceName.snakeCase»'''
	}
	
	override serviceImport(Interface ^interface) {
		return  '''«^interface.interfacePackage»/srv/«^interface.interfaceName.snakeCase»'''
	}

	protected def snakeCase(String input) {
		return CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, input)
	}
	
	override platformType(BasicDataType type)'''«type.typeName»'''
	private def dispatch String typeName(BooleanDataType type)'''std_msgs::msg::Bool'''
	private def dispatch String typeName(StringDataType type)'''std_msgs::msg::String'''
	private def dispatch String typeName(DoubleDataType type)'''std_msgs::msg::Float32'''
	private def dispatch String typeName(IntegerDataType type)'''std_msgs::msg::Int16'''
	private def dispatch String typeName(NoneDataType type)'''std_msgs::msg::Empty'''
	
	override informationPublisher()'''
		auto msg = std_msgs::msg::String();
		msg.data = output.str();
		
		this->state_information->publish(msg);
	'''
}