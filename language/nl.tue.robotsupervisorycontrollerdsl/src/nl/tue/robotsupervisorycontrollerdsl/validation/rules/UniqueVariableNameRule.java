package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Automaton;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AutomatonVariable;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class UniqueVariableNameRule extends AbstractValidationRule {
	public static final String DUPLICATE_VARIABLE_NAME = "duplicateVariableName";

	@Check
	public void checkUniqueVariableName(Variable variable) {
		Automaton automaton = ModelHelper.findParentOfType(variable, Automaton.class);

		List<Variable> equalNames = automaton
				.getDefinitions()
				.stream()
				.filter(it -> it instanceof AutomatonVariable)
				.map(it -> ((AutomatonVariable) it).getVariable())
				.filter(otherVariable -> variable.getName().toLowerCase().equals(otherVariable.getName().toLowerCase()))
				.collect(Collectors.toList());
	
		if (equalNames.size() > 1) {
			error("Variable names should be unique within the automaton.",
					RobotSupervisoryControllerDSLPackage.Literals.VARIABLE__NAME,
					DUPLICATE_VARIABLE_NAME);
		}
	}
}
