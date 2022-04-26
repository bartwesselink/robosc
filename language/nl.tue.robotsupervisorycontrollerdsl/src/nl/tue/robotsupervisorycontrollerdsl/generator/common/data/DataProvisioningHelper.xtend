package nl.tue.robotsupervisorycontrollerdsl.generator.common.data

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement

@Singleton
class DataProvisioningHelper {
	def provideStatements(CommunicationType communicationType, Robot robot) {
		return robot.definitions
			.filter(ProvideStatement)
			.filter[it.communicationType == communicationType]
	}
}