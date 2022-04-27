package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComponentBehaviour;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LocalComponent;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class SingleComponentBehaviourRule extends AbstractValidationRule {
	public static final String MULTIPLE_COMPONENT_BEHAVIOUR = "multipleComponentBehaviours";

	@Check
	public void checkSingleComponentBehaviourDefinition(LocalComponent entity) {
		List<ComponentBehaviour> behaviours = entity
				.getDefinitions()
				.stream()
				.filter(definition -> definition instanceof ComponentBehaviour)
				.map(definition -> (ComponentBehaviour) definition)
				.collect(Collectors.toList());
		
		if (behaviours.size() > 1) {
			error("A component can only have a single behaviour definition.",
					RobotSupervisoryControllerDSLPackage.Literals.COMPONENT_BEHAVIOUR__AUTOMATON,
					MULTIPLE_COMPONENT_BEHAVIOUR);
		}
	}
}
