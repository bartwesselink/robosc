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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueVariableNameRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class UniqueVariableNameRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				component Component1 {
					behaviour {
						variable one: boolean = false
						variable two: boolean = false
						initial state one {}
					}
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkError() {
		"
			robot UnitTestRobot {
				component Component1 {
					behaviour {
						variable one: boolean = false
						variable one: boolean = false

						initial state one {}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.VARIABLE,
			UniqueVariableNameRule.DUPLICATE_VARIABLE_NAME
		)
	}
}
