package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class UniqueDataTypeNameRule extends AbstractValidationRule {
	public static final String DUPLICATE_DATA_TYPE = "duplicateDataType";

	@Check
	public void checkUniqueDataTypeName(ComplexDataType dataType) {
		Robot robot = ModelHelper.findParentOfType(dataType, Robot.class);

		List<ComplexDataType> equalNames = robot
				.getDefinitions()
				.stream()
				.filter(definition -> definition instanceof ComplexDataType)
				.map(definition -> (ComplexDataType) definition)
				.filter(otherType -> otherType.getName().toLowerCase().equals(dataType.getName().toLowerCase()))
				.collect(Collectors.toList());
		
		if (equalNames.size() > 1) {
			error("Data types names should be unique.",
					RobotSupervisoryControllerDSLPackage.Literals.COMPLEX_DATA_TYPE__NAME,
					DUPLICATE_DATA_TYPE);
		}
	}
}
