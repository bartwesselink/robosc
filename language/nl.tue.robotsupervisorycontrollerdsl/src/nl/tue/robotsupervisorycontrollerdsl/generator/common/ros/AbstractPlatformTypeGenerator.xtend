package nl.tue.robotsupervisorycontrollerdsl.generator.common.ros

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Interface
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType

abstract class AbstractPlatformTypeGenerator {	
	abstract def CharSequence messageType(DataType data, Interface ^interface)
	abstract def CharSequence actionType(Interface ^interface)
	abstract def CharSequence serviceType(Interface ^interface)
	abstract def CharSequence messageImport(Interface ^interface)
	abstract def CharSequence actionImport(Interface ^interface)
	abstract def CharSequence serviceImport(Interface ^interface)
	abstract def String platformType(BasicDataType dataType)
	abstract def String informationPublisher()
	abstract def String enumPointerType()
}