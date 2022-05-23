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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.InterfaceLinkRequiredRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class InterfaceLinkRequiredRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				interface test use name from package

				datatype object Complex {
                  importedValue: boolean
				}

				component Component {
					outgoing message distance with type: Complex links test

					behaviour {
						initial state idle {}
					}
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkCommunicationTypeImportError() {
		"
			robot UnitTestRobot {
				datatype object Complex {
                  importedValue: boolean
				}

				component Component {
					outgoing message distance with type: Complex

					behaviour {
						initial state idle {}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE,
			InterfaceLinkRequiredRule.TYPE_SETTINGS_REQUIRED
		)
	}
}
