package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageTo;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResultTransition;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class NoAssignmentOnMessagesToNodeRule extends AbstractValidationRule {
	public static final String NO_ASSIGNMENT_ON_MESSAGS_TO_NODE = "noAssignmentOnMessagesToNode";

	@Check
	public void checkAssignmentOnMessagesToNode(ResultTransition entity) {
		CommunicationType communication = entity.getCommunicationType();
		
		if (!(communication instanceof Message)) return;
		
		Message message = (Message) communication;
		
		if (!(message.getDirection() instanceof MessageTo)) return;
		
		if (entity.getAssignment() != null) {
			error("No assignment can be performed when sending messages to a component.",
					RobotSupervisoryControllerDSLPackage.Literals.RESULT_TRANSITION__ASSIGNMENT,
					NO_ASSIGNMENT_ON_MESSAGS_TO_NODE);
		}
	}
}
