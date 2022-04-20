package nl.tue.robotsupervisorycontrollerdsl.generator.common.data;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty;

public class PropertyIdentifierPair {
	private ObjectProperty property;
	private String identifier;
	
	public PropertyIdentifierPair(ObjectProperty property, String identifier) {
		super();
		this.property = property;
		this.identifier = identifier;
	}

	public ObjectProperty getProperty() {
		return property;
	}

	public void setProperty(ObjectProperty property) {
		this.property = property;
	}

	public String getIdentifier() {
		return identifier;
	}

	public void setIdentifier(String identifier) {
		this.identifier = identifier;
	}
}
