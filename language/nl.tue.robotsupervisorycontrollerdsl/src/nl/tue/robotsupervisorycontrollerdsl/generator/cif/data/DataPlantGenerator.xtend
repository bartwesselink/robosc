package nl.tue.robotsupervisorycontrollerdsl.generator.cif.data

import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.StateNames
import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.expressions.ExpressionGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.TransitionNames

@Singleton
class DataPlantGenerator {
	@Inject extension PlantNames
	@Inject extension StateNames
	@Inject extension TransitionNames
	@Inject extension ExpressionGenerator

	def compileDataplant(CommunicationType communicationType, Robot robot)'''
	plant «communicationType.dataPlantName»:
		«IF !communicationType.provideStatements(robot).empty»
		controllable «communicationType.provideStatements(robot).map[it.transitionName].join(', ')»;
		«ENDIF»
	
		location none:
			initial; marked;
			«communicationType.transitions(robot)»
		«FOR statement : communicationType.provideStatements(robot)»
		location «statement.dataLocationName»:
			«communicationType.transitions(robot)»
		«ENDFOR»
	end
	'''
	
	private def transitions(CommunicationType communicationType, Robot robot)'''
	«FOR statement : communicationType.provideStatements(robot)»
	edge «statement.transitionName» «IF statement.expression !== null»when «statement.expression?.compile»«ENDIF» goto «statement.dataLocationName»;
	«ENDFOR»
	'''
	
	private def provideStatements(CommunicationType communicationType, Robot robot) {
		return robot.definitions
			.filter(ProvideStatement)
			.filter[it.communicationType == communicationType]
	}
}
