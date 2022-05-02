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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.TypeSettingsRequiredRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class TypeSettingsRequiredRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				datatype object Complex {
                  importedValue: boolean
				}

				datatype enum ComplexEnum from Complex (import name from package) to {
					default -> yes
				}

				component Component {
					incoming message distance with type: Complex (import name from package)

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
					incoming message distance with type: Complex

					behaviour {
						initial state idle {}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE,
			TypeSettingsRequiredRule.TYPE_SETTINGS_REQUIRED
		)
	}

	@Test
	def void checkEnumImportError() {
		"
			robot UnitTestRobot {
				datatype object Complex {
                  importedValue: boolean
				}

				datatype enum ComplexEnum from Complex to {
					default -> yes
				}

				component Component {
					incoming message distance with type: Complex (import name from package)

					behaviour {
						initial state idle {}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.ENUM_DATA_TYPE,
			TypeSettingsRequiredRule.TYPE_SETTINGS_REQUIRED
		)
	}
}
