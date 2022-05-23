package nl.tue.robotsupervisorycontrollerdsl.generator.cif

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.common.GeneratorInterface
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import org.eclipse.xtext.generator.IFileSystemAccess2
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.components.ComponentGenerator
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.data.EnumDataTypeGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.requirements.RequirementGenerator
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Requirement
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.FileHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config

@Singleton
class CifGenerator implements GeneratorInterface {
	@Inject extension ComponentGenerator
	@Inject extension EnumDataTypeGenerator
	@Inject extension RequirementGenerator
	@Inject CifSynthesisTool cifSynthesisTool 

	override generate(Robot robot, IFileSystemAccess2 fileSystemAccess, Config config) {
		val fileName = '''«robot.name»/controller.cif'''

		fileSystemAccess.generateFile(fileName, robot.controller)
		
		val path = FileHelper.findAbsolutePath(fileName, fileSystemAccess, robot.eResource.resourceSet)
		cifSynthesisTool.applySynthesis(path)
	}

	def controller(Robot robot) '''
	// Component definitions
	«FOR component : robot.definitions.filter(Component)»«component.compile(robot)»«ENDFOR»
	
	// Data type definitions
	«FOR dataType : robot.definitions.filter(EnumDataType)»
	«dataType.compile»
	«ENDFOR»
			
	// Requirements
	«FOR requirement : robot.definitions.filter(Requirement)»«requirement.compile»«ENDFOR»
	'''
}
