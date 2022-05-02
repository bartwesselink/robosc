package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LocalComponent;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class UniqueCommunicationTypeNameRule extends AbstractValidationRule {
	public static final String DUPLICATE_COMMUNICATION_TYPE = "duplicateCommunicationType";

	@Check
	public void checkUniqueCommunicationTypeName(CommunicationType type) {
		Robot robot = ModelHelper.findParentOfType(type, Robot.class);

		List<CommunicationType> equalNames = robot
				.getDefinitions()
				.stream()
				.filter(definition -> definition instanceof Component)
				.map(component -> (Component) component)
				.filter(component -> component.getType() instanceof LocalComponent)
				.map(component -> (LocalComponent) component.getType())
				.flatMap(local -> local.getDefinitions().stream())
				.filter(definition -> definition instanceof CommunicationType)
				.map(definition -> (CommunicationType) definition)
				.filter(otherType -> type.getName().toLowerCase().equals(otherType.getName().toLowerCase()))
				.collect(Collectors.toList());
	
		if (equalNames.size() > 1) {
			error("Communication names should be unique.",
					RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE__NAME,
					DUPLICATE_COMMUNICATION_TYPE);
		}
	}
}
