package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Automaton;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TransitionStateChange;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class MarkedStateRule extends AbstractValidationRule {
	public static final String NOT_ALL_REACH_MARKED_STATE = "notAllReachMarkedState";

	@Check
	public void checkMarkedState(Automaton entity) {
		// Start from the initial state, and find all reachable states to see if a marked state can be reached
		
		List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> initialStates = entity
				.getDefinitions()
				.stream()
				.filter(definition -> definition instanceof nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State)
				.map(definition -> (nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State) definition)
				.filter(state -> state.isInitial())
				.collect(Collectors.toList());
		
		if (initialStates.size() == 1) {			
			List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> reachableStates = new ArrayList<>();
			this.fillReachableStates(initialStates.get(0), reachableStates);
			
			for (nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State reachable : reachableStates) { 
				List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> seen = new ArrayList<>();

				if (!this.hasReachableMarkedState(reachable, seen)) {
					error("The automaton has reachable states that can not reach a marked state.",
							RobotSupervisoryControllerDSLPackage.Literals.AUTOMATON__DEFINITIONS,
							NOT_ALL_REACH_MARKED_STATE);
				}
			}
		}
	}
	
	private void fillReachableStates(
			nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State current,
			List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> result
		) {
		
		List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> stateChanges =
				ModelHelper.findChildren(current, TransitionStateChange.class)
				.stream()
				.map(it -> it.getState())
				.collect(Collectors.toList());
		
		for (nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State otherState : stateChanges) {
			if (!result.contains(otherState)) {
				result.add(otherState);
				this.fillReachableStates(otherState, result);
			}

		}
	}
	
	private boolean hasReachableMarkedState(
			nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State current,
			List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> seen
		) {

		if (current.isMarked()) {
			return true;
		}
		
		seen.add(current);

		List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> stateChanges =
				ModelHelper.findChildren(current, TransitionStateChange.class)
				.stream()
				.map(it -> it.getState())
				.collect(Collectors.toList());
		
		for (nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State otherState : stateChanges) {
			if (seen.contains(otherState)) {
				continue;
			}
			
			List<nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State> copy = new ArrayList<>(seen);
			
			if (this.hasReachableMarkedState(otherState, copy)) {
				return true;
			}
		}
		
		return false;
	}
}
