package nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.ProvideStatementNamer
import javax.inject.Inject

@Singleton
class StateNames {	
	@Inject ProvideStatementNamer provideStatementNamer

	def dataLocationName(ProvideStatement statement)'''data_«provideStatementNamer.getName(statement)»'''
}