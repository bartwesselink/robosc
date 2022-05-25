package nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.remapping;

import java.util.List;

import javax.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.AbstractUniqueModelNamer;
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;

@Singleton
public class SupervisorMappingService extends AbstractUniqueModelNamer<String> {	
	public List<CommunicationType> getAllRemappedCommunicationTypes(Robot robot) {
		return ModelHelper.findWithinRobot(robot, CommunicationType.class);
	}
}
