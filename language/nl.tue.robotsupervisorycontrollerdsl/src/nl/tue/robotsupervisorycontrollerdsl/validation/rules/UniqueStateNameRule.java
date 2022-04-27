package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Automaton;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class UniqueStateNameRule extends AbstractValidationRule {
	public static final String DUPLICATE_STATE_NAME = "duplicateStateName";

	@Check
	public void checkUniqueStateName(nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State state) {
		Automaton automaton = ModelHelper.findParentOfType(state, Automaton.class);

		List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> equalNames = automaton
				.getDefinitions()
				.stream()
				.filter(definition -> definition instanceof nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State)
				.map(definition -> (nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State) definition)
				.filter(otherState -> state.getName().toLowerCase().equals(otherState.getName().toLowerCase()))
				.collect(Collectors.toList());
		
		if (equalNames.size() > 1) {
			error("State names should be unique.",
					RobotSupervisoryControllerDSLPackage.Literals.STATE__NAME,
					DUPLICATE_STATE_NAME);
		}
	}
}
