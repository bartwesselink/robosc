package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CancelResultType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.FeedbackResultType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageTo;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RequestResultType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResponseResultType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResultTransition;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class CorrectResultTypeRule extends AbstractValidationRule {
	public static final String NO_SERVICE_FEEDBACK = "noServiceFeedback";
	public static final String MESSAGE_RESPONSE_ONLY = "messageResponseOnly";
	public static final String MESSAGE_REQUEST_ONLY = "messageRequestOnly";

	@Check
	public void checkCorrectResultType(ResultTransition entity) {
		CommunicationType communication = entity.getCommunicationType();
		
		if (communication instanceof Service) {
			if (entity.getResultType() instanceof FeedbackResultType || entity.getResultType() instanceof CancelResultType) {
				error("A service does not provide feedback.",
						RobotSupervisoryControllerDSLPackage.Literals.RESULT_TRANSITION__RESULT_TYPE,
						NO_SERVICE_FEEDBACK);
			}
		}
		
		if (communication instanceof Message) {
			if (((Message) communication).getDirection() instanceof MessageFrom) {
				if (!(entity.getResultType() instanceof ResponseResultType)) {
					error("A message coming from a node can only receive a response.",
							RobotSupervisoryControllerDSLPackage.Literals.RESULT_TRANSITION__RESULT_TYPE,
							MESSAGE_RESPONSE_ONLY);
				}
			}

			if (((Message) communication).getDirection() instanceof MessageTo) {
				if (!(entity.getResultType() instanceof RequestResultType)) {
					error("A message coming from a node can only sent a request.",
							RobotSupervisoryControllerDSLPackage.Literals.RESULT_TRANSITION__RESULT_TYPE,
							MESSAGE_REQUEST_ONLY);
				}
			}			
		}
	}
}
