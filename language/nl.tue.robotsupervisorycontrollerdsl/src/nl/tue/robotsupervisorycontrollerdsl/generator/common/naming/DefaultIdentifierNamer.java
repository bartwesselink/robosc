package nl.tue.robotsupervisorycontrollerdsl.generator.common.naming;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;

public class DefaultIdentifierNamer implements IdentifierNamerInterface {
	@Override
	public String name(CommunicationType item) {
		if (item.getIdentifier() != null) {
			return item.getIdentifier();
		} else {
			return item.getName();
		}
	}
}
