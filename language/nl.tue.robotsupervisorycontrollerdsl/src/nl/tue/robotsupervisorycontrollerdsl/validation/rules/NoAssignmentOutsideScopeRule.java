package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AccessibleItem;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Assignment;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class NoAssignmentOutsideScopeRule extends AbstractValidationRule {
	public static final String NO_ASSIGNMENT_OUTSIDE_SCOPE = "noAssignmentOutsideScope";

	@Check
	public void checkNoAssignmentOutsideScope(Assignment entity) {
		Access variable = entity.getItem();
		AccessibleItem first = variable.getFirstItem();
		
		Component parentComponent = ModelHelper.findParentOfType(entity, Component.class);
		
		if (first instanceof Component && first != parentComponent) {
			error("No assignment can happen to a variable outside of this scope.",
					RobotSupervisoryControllerDSLPackage.Literals.ASSIGNMENT__ITEM,
					NO_ASSIGNMENT_OUTSIDE_SCOPE);
		}
	}
}
