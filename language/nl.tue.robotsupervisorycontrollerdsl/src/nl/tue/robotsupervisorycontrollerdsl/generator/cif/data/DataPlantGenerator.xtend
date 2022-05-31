package nl.tue.robotsupervisorycontrollerdsl.generator.cif.data

import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.StateNames
import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.expressions.ExpressionGenerator
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.DataProvisioningHelper

@Singleton
class DataPlantGenerator {
	@Inject extension PlantNames
	@Inject extension StateNames
	@Inject extension TransitionNames
	@Inject extension ExpressionGenerator
	@Inject extension DataProvisioningHelper

	def compileDataplant(CommunicationType communicationType, Robot robot)'''
	plant «communicationType.dataPlantName»:
		controllable c_none«IF !communicationType.provideStatements(robot).empty», «communicationType.provideStatements(robot).map[it.transitionName].join(', ')»«ENDIF»;
	
		location none:
			initial; marked;
			«communicationType.transitions(robot)»
		«FOR statement : communicationType.provideStatements(robot)»
		location «statement.dataLocationName»:
			marked;
			«communicationType.transitions(robot)»
		«ENDFOR»
	end
	'''
	
	private def transitions(CommunicationType communicationType, Robot robot)'''
	edge c_none «IF !communicationType.guards(robot).empty»when «communicationType.guards(robot).join(' and ')» «ENDIF»goto none;
	«FOR statement : communicationType.provideStatements(robot)»
	edge «statement.transitionName» «IF statement.expression !== null»when «statement.expression?.compile»«ENDIF» goto «statement.dataLocationName»;
	«ENDFOR»
	'''
	
	private def guards(CommunicationType communicationType, Robot robot) {
		return communicationType.provideStatements(robot)
			.map[it.expression ?: 'true']
			.map['''not («it»)''']
	}
}
