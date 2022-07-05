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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.ValidEnumValueRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class ValidEnumValueRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				datatype enum UnitTestEnum from integer to {
					value > 0 -> positive
					default -> negative
				}

				component Component1 {
					behaviour {
						initial marked state unique {}
					}
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkStateOverlapError() {
		"
			robot UnitTestRobot {
				datatype enum UnitTestEnum from integer to {
					value > 0 -> positive
					default -> negative
				}

				component Component1 {
					behaviour {
						initial marked state positive {}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.ENUM_VALUE,
			ValidEnumValueRule.INVALID_ENUM_VALUE
		)
	}

	@Test
	def void checkUniqueError() {
		"
			robot UnitTestRobot {
				datatype enum UnitTestEnum from integer to {
					value > 0 -> positive
					default -> positive
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.ENUM_VALUE,
			ValidEnumValueRule.INVALID_ENUM_VALUE
		)
	}
}
