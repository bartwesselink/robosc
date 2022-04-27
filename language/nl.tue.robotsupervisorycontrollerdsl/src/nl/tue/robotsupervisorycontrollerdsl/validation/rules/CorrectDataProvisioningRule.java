package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class CorrectDataProvisioningRule extends AbstractValidationRule {
	public static final String NO_MESSAGE_DATA_FROM_NODE = "noMessageDataFromNode";

	@Check
	public void checkCorrectProvisioning(ProvideStatement entity) {
		CommunicationType communication = entity.getCommunicationType();
		
		if (communication instanceof Message) {
			if (((Message) communication).getDirection() instanceof MessageFrom) {
				error("A message coming from a node can not be provided with data.",
						RobotSupervisoryControllerDSLPackage.Literals.PROVIDE_STATEMENT__COMMUNICATION_TYPE,
						NO_MESSAGE_DATA_FROM_NODE);
			}
		}
	}
}
