package nl.tue.robotsupervisorycontrollerdsl.validation;

import org.eclipse.xtext.validation.ComposedChecks;

import nl.tue.robotsupervisorycontrollerdsl.validation.rules.CorrectDataProvisioningRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.CorrectResultTypeRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.IntegerRangeRequiredRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.NoArrayAllowedRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.NoAssignmentOnMessagesToNodeRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.NoAssignmentOutsideScopeRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.NoObjectVariableTypeRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.SingleComponentBehaviourRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.SingleDefaultEnumRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.SingleInitialStateRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.TypeCheckRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.TypeSettingsRequiredRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueCommunicationTypeNameRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueComponentNameRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueDataTypeNameRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueStateNameRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueTopicNameRule;

@ComposedChecks(validators = {
		SingleInitialStateRule.class,
		UniqueCommunicationTypeNameRule.class,
		UniqueDataTypeNameRule.class,
		UniqueStateNameRule.class,
		SingleComponentBehaviourRule.class,
		SingleDefaultEnumRule.class,
		NoAssignmentOnMessagesToNodeRule.class,
		NoAssignmentOutsideScopeRule.class,
		CorrectResultTypeRule.class,
		CorrectDataProvisioningRule.class,
		TypeCheckRule.class,
		UniqueTopicNameRule.class,
		NoObjectVariableTypeRule.class,
		IntegerRangeRequiredRule.class,
		TypeSettingsRequiredRule.class,
		NoArrayAllowedRule.class,
		UniqueComponentNameRule.class,
})
public class RobotSupervisoryControllerDSLValidator extends AbstractRobotSupervisoryControllerDSLValidator {
}
