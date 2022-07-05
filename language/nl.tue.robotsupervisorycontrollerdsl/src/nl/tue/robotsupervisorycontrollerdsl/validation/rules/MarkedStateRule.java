package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Automaton;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class MarkedStateRule extends AbstractValidationRule {
	public static final String NO_MARKED_STATE = "noMarkedState";

	@Check
	public void checkMarkedState(Automaton entity) {
		List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> initialStates = entity
				.getDefinitions()
				.stream()
				.filter(definition -> definition instanceof nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State)
				.map(definition -> (nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State) definition)
				.filter(state -> state.isMarked())
				.collect(Collectors.toList());
		
		if (initialStates.isEmpty()) {
			error("An automaton should have at least a marked state.",
					RobotSupervisoryControllerDSLPackage.Literals.AUTOMATON__DEFINITIONS,
					NO_MARKED_STATE);
		}
	}
}
