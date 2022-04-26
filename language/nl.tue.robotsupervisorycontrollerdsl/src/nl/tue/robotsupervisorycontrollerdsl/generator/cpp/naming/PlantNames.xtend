package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming

import javax.inject.Singleton
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType

@Singleton
class PlantNames {
	@Inject nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames cifPlantNames

	def dataPlantName(CommunicationType communicationType) '''«cifPlantNames.dataPlantName(communicationType)»_'''	
}

