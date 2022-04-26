package nl.tue.robotsupervisorycontrollerdsl.generator.common.ros

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CustomTypeSettings
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType

abstract class AbstractPlatformTypeGenerator {	
	abstract def CharSequence messageType(DataType data, CustomTypeSettings settings)
	abstract def CharSequence actionType(CustomTypeSettings settings)
	abstract def CharSequence serviceType(CustomTypeSettings settings)
	abstract def CharSequence messageImport(CustomTypeSettings settings)
	abstract def CharSequence actionImport(CustomTypeSettings settings)
	abstract def CharSequence serviceImport(CustomTypeSettings settings)
	abstract def String platformType(BasicDataType dataType)
}