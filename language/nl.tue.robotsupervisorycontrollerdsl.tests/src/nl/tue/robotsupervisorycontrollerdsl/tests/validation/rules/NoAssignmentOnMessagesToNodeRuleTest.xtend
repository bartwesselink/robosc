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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.NoAssignmentOnMessagesToNodeRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class NoAssignmentOnMessagesToNodeRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message with type: boolean

					behaviour {
						variable result: boolean = false

						initial state idle {
							on response from test_message do result := value		
						}
					}
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkMessageRequestAssignment() {
		"
			robot UnitTestRobot {
				component Component {
					incoming message test_message with type: boolean

					behaviour {
						variable result: boolean = false

						initial state idle {
							on request to test_message do result := value		
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.RESULT_TRANSITION,
			NoAssignmentOnMessagesToNodeRule.NO_ASSIGNMENT_ON_MESSAGS_TO_NODE
		)
	}
}
