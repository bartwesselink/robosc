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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.CorrectResultTypeRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class CorrectResultTypeRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				interface unit use Unit from package

				component Component {
					incoming message test_message with type: boolean
					service test_service with request: boolean, response: boolean links unit

					behaviour {
						initial marked state idle {
							on request to test_message		
							on request to test_service	
							on response from test_service		
						}
					}
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkServiceFeedbackError() {
		"
			robot UnitTestRobot {
				interface unit use Unit from package

				component Component {
					incoming message test_message with type: boolean
					service test_service with request: boolean, response: boolean links unit

					behaviour {
						initial marked state idle {
							on feedback from test_service		
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.RESULT_TRANSITION,
			CorrectResultTypeRule.NO_SERVICE_FEEDBACK
		)
	}

	@Test
	def void checkMessageResponseError() {
		"
			robot UnitTestRobot {
				interface unit use Unit from package

				component Component {
					outgoing message test_message with type: boolean
					service test_service with request: boolean, response: boolean links unit

					behaviour {
						initial marked state idle {
							on request to test_message	
						}	
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.RESULT_TRANSITION,
			CorrectResultTypeRule.MESSAGE_RESPONSE_ONLY
		)
	}

	@Test
	def void checkMessageRequestError() {
		"
			robot UnitTestRobot {
				interface unit use Unit from package
	
				component Component {
					incoming message test_message with type: boolean
					service test_service with request: boolean, response: boolean links unit

					behaviour {
						initial marked state idle {
							on response to test_message		
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.RESULT_TRANSITION,
			CorrectResultTypeRule.MESSAGE_REQUEST_ONLY
		)
	}
}
