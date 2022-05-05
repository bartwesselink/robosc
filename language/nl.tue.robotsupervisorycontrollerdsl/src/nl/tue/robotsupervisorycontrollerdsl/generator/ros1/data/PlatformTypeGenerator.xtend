package nl.tue.robotsupervisorycontrollerdsl.generator.ros1.data

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CustomTypeSettings
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractPlatformTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BooleanDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.StringDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DoubleDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.IntegerDataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.NoneDataType

@Singleton
class PlatformTypeGenerator extends AbstractPlatformTypeGenerator {	
	override String messageType(DataType data, CustomTypeSettings settings) {
		if (data instanceof ComplexDataTypeReference) {
			val referenced = data.type
			
			if (referenced instanceof EnumDataType) {
				return referenced.type.messageType(referenced.typeSettings)
			} else {
				return '''«settings.package»::«settings.name»'''
			}
		} else if (data instanceof BasicDataType) {
			return data.platformType
		}
	}
	
	override actionType(CustomTypeSettings settings) {
		return  '''«settings.package»::«settings.name»'''
	}
	
	override serviceType(CustomTypeSettings settings) {
		return  '''«settings.package»::«settings.name»Action'''
	}
	
	override String messageImport(CustomTypeSettings settings) {
		return '''«settings.package»/«settings.name»'''
	}
	
	override actionImport(CustomTypeSettings settings) {
		return  '''«settings.package»/«settings.name»Action'''
	}
	
	override serviceImport(CustomTypeSettings settings) {
		return  '''«settings.package»/«settings.name»'''
	}
	
	override platformType(BasicDataType type)'''«type.typeName»'''
	private def dispatch String typeName(BooleanDataType type)'''std_msgs::Bool'''
	private def dispatch String typeName(StringDataType type)'''std_msgs::String'''
	private def dispatch String typeName(DoubleDataType type)'''std_msgs::Float32'''
	private def dispatch String typeName(IntegerDataType type)'''std_msgs::Int16'''
	private def dispatch String typeName(NoneDataType type)'''std_msgs::Empty'''
	
	override informationPublisher()'''
		std_msgs::String msg;
		msg.data = output.str();
		
		this->state_information.publish(msg);
	'''
}