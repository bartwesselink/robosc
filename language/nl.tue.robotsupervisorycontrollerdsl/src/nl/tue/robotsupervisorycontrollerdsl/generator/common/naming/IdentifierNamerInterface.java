package nl.tue.robotsupervisorycontrollerdsl.generator.common.naming;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;

public interface IdentifierNamerInterface {
	String name(CommunicationType item);
}
