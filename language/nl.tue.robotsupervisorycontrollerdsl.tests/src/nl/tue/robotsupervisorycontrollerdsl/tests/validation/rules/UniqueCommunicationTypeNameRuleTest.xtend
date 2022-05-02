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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueCommunicationTypeNameRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class UniqueCommunicationTypeNameRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				component Component1 {
					incoming message test_message_one with type: boolean
					incoming message test_message_two with type: boolean
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkError() {
		"
			robot UnitTestRobot {
				component Component1 {
					incoming message test_message with type: boolean
					incoming message test_message with type: boolean
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE,
			UniqueCommunicationTypeNameRule.DUPLICATE_COMMUNICATION_TYPE
		)
	}
}
