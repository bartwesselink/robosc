package nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.ros2.remapping;

import javax.inject.Inject;
import javax.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.DefaultIdentifierNamer;
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.IdentifierNamerInterface;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;

@Singleton
public class SupervisorMappingNamer implements IdentifierNamerInterface {
	@Inject private SupervisorMappingService mappingService;
	@Inject private DefaultIdentifierNamer defaultIdentifierNamer;
	
	@Override
	public String name(CommunicationType item) {
		String originalName = defaultIdentifierNamer.name(item);
		
		return originalName + "_" + this.mappingService.getName(originalName);
	}
}
