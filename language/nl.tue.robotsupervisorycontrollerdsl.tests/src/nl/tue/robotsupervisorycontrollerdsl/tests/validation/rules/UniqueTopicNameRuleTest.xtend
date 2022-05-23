package nl.tue.robotsupervisorycontrollerdsl.tests.validation.rules

import com.google.inject.Inject
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.testing.util.ParseHelper
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Base
import nl.tue.robotsupervisorycontrollerdsl.tests.RobotSupervisoryControllerDSLInjectorProvider
import org.eclipse.xtext.testing.validation.ValidationTestHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueTopicNameRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class UniqueTopicNameRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				component Component1 {
					outgoing message test_message_one with identifier: \"topic_one\", type: boolean
				}

				component Component2 {
					outgoing message test_message_two with identifier: \"topic_two\", type: boolean
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkWarning() {
		"
			robot UnitTestRobot {
				component Component1 {
					outgoing message test_message_one with identifier: \"topic\", type: boolean
				}

				component Component2 {
					outgoing message test_message_two with identifier: \"topic\", type: boolean
				}
			}
		".parse.assertWarning(
			RobotSupervisoryControllerDSLPackage.Literals.MESSAGE,
			UniqueTopicNameRule.UNIQUE_TOPIC_NAME
		)
	}
}