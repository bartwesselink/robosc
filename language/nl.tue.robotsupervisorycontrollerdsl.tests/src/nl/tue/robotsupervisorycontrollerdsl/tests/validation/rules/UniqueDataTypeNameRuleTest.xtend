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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UniqueDataTypeNameRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class UniqueDataTypeNameRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				datatype object One {
					item: boolean
				}

				datatype object Two {
					item: boolean
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkError() {
		"
			robot UnitTestRobot {
				datatype object One {
					item: boolean
				}

				datatype object One {
					item: boolean
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.COMPLEX_DATA_TYPE,
			UniqueDataTypeNameRule.DUPLICATE_DATA_TYPE
		)
	}
}
