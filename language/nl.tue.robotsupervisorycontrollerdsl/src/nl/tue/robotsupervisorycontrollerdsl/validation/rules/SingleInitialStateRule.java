package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Automaton;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class SingleInitialStateRule extends AbstractValidationRule {
	public static final String NO_INITIAL_STATE = "noInitialState";
	public static final String MULTIPLE_INITIAL_STATES = "multipleInitialStates";

	@Check
	public void checkOneInitialState(Automaton entity) {
		List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> initialStates = entity
				.getDefinitions()
				.stream()
				.filter(definition -> definition instanceof nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State)
				.map(definition -> (nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State) definition)
				.filter(state -> state.isInitial())
				.collect(Collectors.toList());
		
		if (initialStates.isEmpty()) {
			error("An automaton should have one initial state.",
					RobotSupervisoryControllerDSLPackage.Literals.AUTOMATON__DEFINITIONS,
					NO_INITIAL_STATE);
		} else if (initialStates.size() > 1) {
			error("An automaton can not have multiple initial states.",
					RobotSupervisoryControllerDSLPackage.Literals.AUTOMATON__DEFINITIONS,
					MULTIPLE_INITIAL_STATES);
		}
	}
}
