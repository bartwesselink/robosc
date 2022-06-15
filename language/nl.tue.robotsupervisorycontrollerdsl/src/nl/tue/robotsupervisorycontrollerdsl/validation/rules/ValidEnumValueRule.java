package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumValue;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Library;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class ValidEnumValueRule extends AbstractValidationRule {
	public static final String INVALID_ENUM_VALUE = "invalidEnumValue";

	@Check
	public void checkValidEnumValue(EnumValue value) {
		Robot robot = ModelHelper.findParentOfType(value, Robot.class);
		Library library = ModelHelper.findParentOfType(value, Library.class);
		
		List<EnumValue> allEnumValues = new ArrayList<>();
		List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> allStates = new ArrayList<>();

		if (robot != null) {
			allEnumValues.addAll(ModelHelper.findWithinRobot(robot, EnumValue.class));
			allStates.addAll(ModelHelper.findWithinRobot(robot, nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State.class));
		} else if (library != null) {
			allEnumValues.addAll(ModelHelper.findChildren(library, EnumValue.class));
			allStates.addAll(ModelHelper.findWithinRobot(robot, nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State.class));
		}

		List<EnumValue> equalNames = allEnumValues
				.stream()
				.filter(otherEnumValue -> value.getName().toLowerCase().equals(otherEnumValue.getName().toLowerCase()))
				.collect(Collectors.toList());
		
		// State names and enum value names can not overlap
		List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> equalStateNames = allStates
				.stream()
				.filter(otherState -> value.getName().toLowerCase().equals(otherState.getName().toLowerCase()))
				.collect(Collectors.toList());
			
		if (equalNames.size() > 1) {
			error("Enum value names should be unique.",
					RobotSupervisoryControllerDSLPackage.Literals.ENUM_VALUE__NAME,
					INVALID_ENUM_VALUE);
		}
		
		if (equalStateNames.size() > 0) {
			error("State names cannot overlap with enum value names.",
					RobotSupervisoryControllerDSLPackage.Literals.ENUM_VALUE__NAME,
					INVALID_ENUM_VALUE);
		}
	}
}
