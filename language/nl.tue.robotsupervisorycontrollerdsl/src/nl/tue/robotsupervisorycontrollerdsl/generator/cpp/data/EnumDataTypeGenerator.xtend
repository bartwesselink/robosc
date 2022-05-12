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
import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractPlatformTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.ParameterNames
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType

@Singleton
class EnumDataTypeGenerator {
	@Inject extension ExpressionGenerator
	@Inject extension DataTypeGenerator
	@Inject extension EnumHelper
	@Inject extension MethodNames
	@Inject extension ParameterNames
	
	def compile(EnumDataType ^enum, AbstractPlatformTypeGenerator generator)'''
	«enumTypeName» «^enum.convertMethod»(const «enum.parameterInputType(generator)» «enum.sourceInputName») {
		«FOR rule : enum.rules.filter(EnumTransformationRule) SEPARATOR '\n} else '»if («rule.expression.compile») {
			return «rule.value.correspondingEngineType»;«ENDFOR»
		}
	
		return «enum.defaultRule.value.correspondingEngineType»;
	}
	'''

	def correspondingEngineType(EnumValue value) {
		return '''_«CifSynthesisTool.codePrefix»_«value.name»'''
	}
	
	def parameterInputType(EnumDataType ^enum, AbstractPlatformTypeGenerator generator) {
		val type = enum.type
		
		if (type instanceof ComplexDataTypeReference) {
			// Check to see if it is used in a result transition
			val robot = ModelHelper.findParentOfType(enum, Robot)
			
			if (robot !== null) {
				val resultTransitionsWithEnum = ModelHelper.findWithinRobot(robot, CommunicationType)
					.filter[!ModelHelper.findChildren(it, ComplexDataTypeReference)
						.filter[it.type == enum]
						.empty
					]
					
				if (!resultTransitionsWithEnum.empty) {
					val communicationType = resultTransitionsWithEnum.get(0)
					
					if (communicationType instanceof Message) {
						return '''«generator.messageType(communicationType.type, communicationType.typeSettings)»::SharedPtr'''
					} else if (communicationType instanceof Service) {
						return '''«generator.serviceType(communicationType.typeSettings)»::SharedPtr'''
					} else if (communicationType instanceof Action) {
						return '''«generator.actionType(communicationType.typeSettings)»::SharedPtr'''
					}
				}
			}
			
			return '''«enum.typeSettings?.name»::SharedPtr'''
		} else if (type instanceof BasicDataType) {
			return type.compile(generator)
		}
	}
}