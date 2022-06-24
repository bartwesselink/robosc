package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class UniquePropertyNameRule extends AbstractValidationRule {
	public static final String DUPLICATE_PROPERTY_NAME = "duplicatePropertyName";

	@Check
	public void checkUniquePropertyName(ObjectProperty property) {
		ObjectDataType object = ModelHelper.findParentOfType(property, ObjectDataType.class);

		List<ObjectProperty> equalNames = object
				.getProperties()
				.stream()
				.filter(otherProperty -> property.getName().toLowerCase().equals(otherProperty.getName().toLowerCase()))
				.collect(Collectors.toList());
	
		if (equalNames.size() > 1) {
			error("Property names should be unique.",
					RobotSupervisoryControllerDSLPackage.Literals.OBJECT_PROPERTY__NAME,
					DUPLICATE_PROPERTY_NAME);
		}
	}
}
