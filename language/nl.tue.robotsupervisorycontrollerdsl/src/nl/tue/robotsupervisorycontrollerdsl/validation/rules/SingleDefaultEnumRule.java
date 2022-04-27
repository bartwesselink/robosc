package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDefaultRule;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumRule;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class SingleDefaultEnumRule extends AbstractValidationRule {
	public static final String NO_DEFAULT_CASE = "noDefaultCase";
	public static final String MULTIPLE_DEFAULT_CASES = "multipleDefaultCases";

	@Check
	public void checkOneDefaultCase(EnumDataType entity) {
		List<EnumRule> defaultCases = entity
				.getRules()
				.stream()
				.filter(definition -> definition instanceof EnumDefaultRule)
				.map(definition -> (EnumDefaultRule) definition)
				.collect(Collectors.toList());
		
		if (defaultCases.isEmpty()) {
			error("An enum data type must have at least one default transformation rule.",
					RobotSupervisoryControllerDSLPackage.Literals.ENUM_DATA_TYPE__RULES,
					NO_DEFAULT_CASE);
		} else if (defaultCases.size() > 1) {
			error("An enum data type must have at least one default transformation rule.",
					RobotSupervisoryControllerDSLPackage.Literals.ENUM_DATA_TYPE__RULES,
					MULTIPLE_DEFAULT_CASES);
		}
	}
}
