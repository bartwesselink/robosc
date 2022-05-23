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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.UsedDataTypeRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class UsedDataTypeRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				datatype enum TestEnum from double to {
					value > 0.0 -> yes
	                default -> no
				}

				component Component1 {
					outgoing message test_message with type: integer(0..20)
					outgoing message test_message_string with type: string

					behaviour {
						variable result: boolean
						variable result_string: string = \"\"
						variable result_double: double = 0.0

						initial state idle {}
					}
				}
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkCommunicationTypeArrayError() {
		"
			robot UnitTestRobot {
				component Component1 {
					outgoing message test_message with type: array(integer)
					outgoing message test_message_string with type: string

					behaviour {
						variable result: boolean

						initial state idle {}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE,
			UsedDataTypeRule.NO_ARRAY_ALLOWED
		)
	}

	@Test
	def void checkVariableTypeArrayError() {
		"
			robot UnitTestRobot {
				component Component1 {
					outgoing message test_message with type: boolean
					outgoing message test_message_string with type: string

					behaviour {
						variable result: array(boolean)

						initial state idle {}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.VARIABLE,
			UsedDataTypeRule.NO_ARRAY_ALLOWED
		)
	}

	@Test
	def void checkVariableTypeStringError() {
		"
			robot UnitTestRobot {
				component Component1 {
					outgoing message test_message with type: boolean
					outgoing message test_message_string with type: string

					behaviour {
						variable result: string
						variable result_string: string = \"\"
						variable result_double: double = 0.0

						initial state idle {
							transition if result_string = \"\" goto idle
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.VARIABLE,
			UsedDataTypeRule.NO_STRING_ALLOWED
		)
	}

	@Test
	def void checkVariableTypeDoubleError() {
		"
			robot UnitTestRobot {
				component Component1 {
					outgoing message test_message with type: boolean
					outgoing message test_message_string with type: string

					behaviour {
						variable result: string
						variable result_string: string = \"\"
						variable result_double: double = 0.0

						initial state idle {
							transition if result_double > 0.0 goto idle
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.VARIABLE,
			UsedDataTypeRule.NO_DOUBLE_ALLOWED
		)
	}

	@Test
	def void checkEnum() {
		"
			robot UnitTestRobot {
				datatype enum FirstEnum from double to {
					value > 0.0 -> once
	                default -> twice
				}

				datatype enum TestEnum from FirstEnum to {
					value = once -> yes
	                default -> no
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.ENUM_DATA_TYPE,
			UsedDataTypeRule.NO_ENUM_ALLOWED
		)
	}
}
