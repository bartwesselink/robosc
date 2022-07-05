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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.MarkedStateRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class MarkedStateRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				component Component {
					behaviour {
						initial marked state idle {}
					}
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkNoMarkedState() {
		"
			robot UnitTestRobot {
				component Component {
					behaviour {
						initial state idle {}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.AUTOMATON,
			MarkedStateRule.NO_MARKED_STATE
		)
	}
}
