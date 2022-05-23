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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.NoAssignmentOutsideScopeRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class NoAssignmentOutsideScopeRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				component Component1 {
					outgoing message test_message_one with type: boolean

					behaviour {
						variable result: boolean = false

						initial state idle {
							on response from test_message_one do result := value		
						}
					}
				}

				component Component2 {
					outgoing message test_message_two with type: boolean

					behaviour {
						variable result: boolean = false

						initial state idle {
							on response from test_message_two do result := value		
						}
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
					outgoing message test_message_one with type: boolean

					behaviour {
						variable result: boolean = false

						initial state idle {
							on response from test_message_one do Component2.result := value		
						}
					}
				}

				component Component2 {
					outgoing message test_message_two with type: boolean

					behaviour {
						variable result: boolean = false

						initial state idle {
							on response from test_message_two do Component1.result := value		
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.ASSIGNMENT,
			NoAssignmentOutsideScopeRule.NO_ASSIGNMENT_OUTSIDE_SCOPE
		)
	}
}
