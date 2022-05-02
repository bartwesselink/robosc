package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LocalComponent;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.MessageFrom;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class UniqueTopicNameRule extends AbstractValidationRule {
	public static final String UNIQUE_TOPIC_NAME = "uniqueName";

	@Check
	public void checkUniqueCommunicationTypeName(Message type) {
		Robot robot = ModelHelper.findParentOfType(type, Robot.class);
		Component messageComponent = ModelHelper.findParentOfType(type, Component.class);

		String topicName = getTopicName(type);

		List<Message> equalTopics = robot.getDefinitions().stream()
				.filter(definition -> definition instanceof Component)
				.filter(definition -> definition != messageComponent)
				.filter(component -> ((Component) component).getType() instanceof LocalComponent)
				.map(component -> (LocalComponent) ((Component) component).getType())
				.flatMap(local -> local.getDefinitions().stream()).filter(definition -> definition instanceof Message)
				.map(definition -> (Message) definition)
				.filter(message -> message.getDirection() instanceof MessageFrom)
				.filter(message -> getTopicName(message).equals(topicName)).collect(Collectors.toList());

		if (equalTopics.size() > 0) {
			warning("Multiple components are publishing to the same topic. This can cause problems, as no distinction can be made as to from which node the message was published.",
					RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE__IDENTIFIER, UNIQUE_TOPIC_NAME);
		}
	}

	private String getTopicName(Message message) {
		return message.getIdentifier() != null ? message.getIdentifier() : message.getName();
	}
}
