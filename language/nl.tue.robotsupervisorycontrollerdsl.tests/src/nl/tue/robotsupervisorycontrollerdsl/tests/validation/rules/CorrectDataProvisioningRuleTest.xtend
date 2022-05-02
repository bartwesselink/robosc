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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.CorrectDataProvisioningRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class CorrectDataProvisioningRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test with type: boolean
				}

				provide test with true
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkError() {
		"
			robot UnitTestRobot {
				component Component {
					incoming message test with type: boolean
				}

				provide test with true
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.PROVIDE_STATEMENT,
			CorrectDataProvisioningRule.NO_MESSAGE_DATA_FROM_NODE
		)
	}
}
