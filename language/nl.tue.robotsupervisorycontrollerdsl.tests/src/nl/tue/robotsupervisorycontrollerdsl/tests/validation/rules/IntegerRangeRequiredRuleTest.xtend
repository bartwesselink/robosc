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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.IntegerRangeRequiredRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class IntegerRangeRequiredRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message distance with type: integer(0..20)
					incoming message move with type: none

					behaviour {
						variable current_distance: integer(0..20) = 0

						initial marked state idle {
							on response from distance do current_distance := value		
						}
					}
				}

				requirement move needs Component.current_distance > 1
			}
		".parse.assertNoErrors
	}
	
	def void checkVariableHasIntegerRange() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message distance with type: integer(0..20)
					incoming message move with type: none

					behaviour {
						variable current_distance: integer = 0

						initial marked state idle {
							on response from distance do current_distance := value		
						}
					}
				}

				requirement move needs Component.current_distance > 1
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.VARIABLE,
			IntegerRangeRequiredRule.INTEGER_RANGE_REQUIRED
		)
	}
	
	def void checkMessageTypeHasIntegerRange() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message distance with type: integer
					incoming message move with type: none

					behaviour {
						variable current_distance: integer(0..20) = 0

						initial marked state idle {
							on response from distance do current_distance := value		
						}
					}
				}

				requirement move needs Component.current_distance > 1
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.MESSAGE,
			IntegerRangeRequiredRule.INTEGER_RANGE_REQUIRED
		)
	}
	
	def void checkObjectTypeHasIntegerRange() {
		"
			robot UnitTestRobot {
				datatype object Complex {
					distance: integer 
				}

				component Component {
					outgoing message distance with type: Complex (import test from test)
					incoming message move with type: none

					behaviour {
						variable current_distance: integer(0..20) = 0

						initial marked state idle {
							on response from distance do current_distance := value.distance	
						}
					}
				}

				requirement move needs Component.current_distance > 1
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.OBJECT_PROPERTY,
			IntegerRangeRequiredRule.INTEGER_RANGE_REQUIRED
		)
	}
}
