package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Library;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class UniqueComponentNameRule extends AbstractValidationRule {
	public static final String DUPLICATE_COMPONENT_NAME = "duplicateComponentName";

	@Check
	public void checkUniqueComponentName(Component component) {
		Robot robot = ModelHelper.findParentOfType(component, Robot.class);
		Library library = ModelHelper.findParentOfType(component, Library.class);
		
		List<Component> components = new ArrayList<>();
		
		if (robot != null) {
			components.addAll(
						robot.getDefinitions().stream().filter(it -> it instanceof Component).map(it -> (Component) it).collect(Collectors.toList())
					);
		} else if (library != null) {
			components.addAll(
					library.getDefinitions().stream().filter(it -> it instanceof Component).map(it -> (Component) it).collect(Collectors.toList())
				);
		}

		List<Component> equalNames = components
				.stream()
				.filter(otherComponent -> component.getName().toLowerCase().equals(otherComponent.getName().toLowerCase()))
				.collect(Collectors.toList());
		
		if (equalNames.size() > 1) {
			error("Component names should be unique.",
					RobotSupervisoryControllerDSLPackage.Literals.COMPONENT__NAME,
					DUPLICATE_COMPONENT_NAME);
		}
	}
}
