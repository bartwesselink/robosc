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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.NoArrayAllowedRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class NoArrayAllowedRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				component Component {
					incoming message distance with type: array(integer)

					behaviour {
						variable current_distance: array(integer)

						initial state idle {
							on response from distance do current_distance := value		
						}
					}
				}
			}
		".parse.assertNoErrors
	}
	
	def void checkVariableHasNoArrayType() {
		"
			robot UnitTestRobot {
				component Component {
					incoming message distance with type: array(integer)
					outgoing message move with type: none

					behaviour {
						variable current_distance: array(integer)

						initial state idle {
							on response from distance do current_distance := value		
						}
					}
				}

				requirement move needs Component.current_distance[0] > 1
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.VARIABLE,
			NoArrayAllowedRule.NO_ARRAY_ALLOWED
		)
	}
	
	def void checkMessageTypeHasNoArrayType() {
		"
			robot UnitTestRobot {
				component Component {
					incoming message distance with type: array(integer)
					outgoing message move with type: none

					behaviour {
						variable current_distance: array(integer)

						initial state idle {
							on response from distance do current_distance := value[0]		
						}
					}
				}

				requirement move needs Component.current_distance[0] > 1
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.MESSAGE,
			NoArrayAllowedRule.NO_ARRAY_ALLOWED
		)
	}
	
	def void checkObjectTypeHasNoArrayType() {
		"
			robot UnitTestRobot {
				datatype object Complex {
					distance: array(integer) 
				}

				component Component {
					incoming message distance with type: Complex (import test from test)
					outgoing message move with type: none

					behaviour {
						variable current_distance: array(integer) = 0

						initial state idle {
							on response from distance do current_distance := value.distance[0]	
						}
					}
				}

				requirement move needs Component.current_distance[0] > 1
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.OBJECT_PROPERTY,
			NoArrayAllowedRule.NO_ARRAY_ALLOWED
		)
	}
}
