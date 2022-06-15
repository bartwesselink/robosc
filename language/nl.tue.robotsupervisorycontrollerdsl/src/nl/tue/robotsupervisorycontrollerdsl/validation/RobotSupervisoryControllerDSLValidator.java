package nl.tue.robotsupervisorycontrollerdsl.validation;

import org.eclipse.xtext.validation.ComposedChecks;

import nl.tue.robotsupervisorycontrollerdsl.validation.rules.CorrectDataProvisioningRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.CorrectResultTypeRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.IntegerRangeRequiredRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.InterfaceLinkRequiredRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.NoAssignmentOnMessagesToNodeRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.NoAssignmentOutsideScopeRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.SingleComponentBehaviourRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.SingleDefaultEnumRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.SingleInitialStateRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.TypeCheckRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueCommunicationTypeNameRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueComponentNameRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueDataTypeNameRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueStateNameRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueTopicNameRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UsedDataTypeRule;
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.ValidEnumValueRule;

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
		IntegerRangeRequiredRule.class,
		InterfaceLinkRequiredRule.class,
		UsedDataTypeRule.class,
		UniqueComponentNameRule.class,
		ValidEnumValueRule.class,
})
public class RobotSupervisoryControllerDSLValidator extends AbstractRobotSupervisoryControllerDSLValidator {
}
