package nl.tue.robotsupervisorycontrollerdsl.generator.common.data;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty;

@Singleton
public class ObjectHelper {
	public List<PropertyIdentifierPair> flattenProperties(ObjectDataType type, String prefix) {
		List<PropertyIdentifierPair> result = new ArrayList<>();
		
		for (ObjectProperty property : type.getProperties()) {
			if (property.getType() instanceof BasicDataType) {
				result.add(new PropertyIdentifierPair(property, prefix + property.getName()));
			} else if (property.getType() instanceof ObjectDataType) {
				result.addAll(
						flattenProperties((ObjectDataType) property.getType(), prefix + property.getName() + '_')
					);
			}
		}
		
		return result;
	}
}
