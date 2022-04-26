package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming

import javax.inject.Singleton
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement

@Singleton
class StateNames {
	@Inject nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.StateNames cifStateNames

	def dataLocationName(ProvideStatement statement) '''_controller_«cifStateNames.dataLocationName(statement)»'''	
}

