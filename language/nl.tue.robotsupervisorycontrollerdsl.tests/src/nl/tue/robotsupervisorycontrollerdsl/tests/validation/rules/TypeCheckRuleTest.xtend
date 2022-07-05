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
import nl.tue.robotsupervisorycontrollerdsl.validation.rules.TypeCheckRule

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class TypeCheckRuleTest {
	@Inject extension ParseHelper<Base>
	@Inject extension ValidationTestHelper

	@Test
	def void checkValid() {
		"
			robot UnitTestRobot {
				interface test use name from package

				datatype object TestObject {
					result: integer(0..20)
				}

				component Component {
					outgoing message test_message_one with type: integer(0..20)
					incoming message test_message_two with type: TestObject links test
					incoming message test_message_three with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_double: double
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_boolean := true or false		
							on response from test_message_one do result_boolean := true and false
							on response from test_message_one do result_boolean := 5 > 0
							on response from test_message_one do result_boolean := 5 >= 0
							on response from test_message_one do result_boolean := 5 < 0
							on response from test_message_one do result_boolean := 5 <= 0
							on response from test_message_one do result_boolean := 5 <= 0
							on response from test_message_one do result_boolean := !true
							on response from test_message_one do result_integer := -5
							on response from test_message_one do result_integer := 5 * 0
							on response from test_message_one do result_double := 0 / 5
							on response from test_message_one do result_integer := 1 + 2
							on response from test_message_one do result_integer := 1 - 2
							transition if true
						}
					}
				}

				provide test_message_two with { result: 5 }
				provide test_message_three with 5
			}
		".parse.assertNoErrors
	}

	@Test
	def void checkErrorOr() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_boolean := true or 1
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.OR,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorAnd() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_boolean := true and 1
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.AND,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorGreaterThan() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_boolean := 5 > true
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.GREATER_THAN,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorSmallerThan() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_boolean := 5 < true
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.SMALLER_THAN,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorGreaterThanEqual() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_boolean := 5 >= true
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.GREATER_THAN_EQUAL,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorSmallerThanEqual() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_boolean := 5 <= true
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.SMALLER_THAN_EQUAL,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorNegation() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_boolean := !5
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.NEGATION,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorNegative() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_integer := -true
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.NEGATIVE,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorMultiply() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_integer := 5 * true
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.MULTIPLY,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorDivide() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_integer := 5 / true
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.DIVIDE,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorPlus() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_integer := 5 + true
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.PLUS,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorMinus() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_integer := 5 - true
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.MINUS,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorTransitionGuard() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_integer: integer(0..20) = 0
						variable result_boolean: boolean = false

						initial marked state idle {
							transition if 5
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.TRANSITION_GUARD,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorProvideStatement() {
		"
			robot UnitTestRobot {
				component Component {
					incoming message test_message_three with type: integer(0..20)
				}

				provide test_message_three with false
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.PROVIDE_STATEMENT,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorObjectPropertyValue() {
		"
			robot UnitTestRobot {
				interface test use name from package

				datatype object TestObject {
					result: integer(0..20)
				}

				component Component {
					incoming message test_message_two with type: TestObject links test
				}

				provide test_message_two with { result: true }
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.OBJECT_PROPERTY_VALUE,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorAssignment() {
		"
			robot UnitTestRobot {
				component Component {
					outgoing message test_message_one with type: integer(0..20)

					behaviour {
						variable result_boolean: boolean = false

						initial marked state idle {
							on response from test_message_one do result_boolean := 5
						}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.ASSIGNMENT,
			TypeCheckRule.INVALID_TYPE
		)
	}

	@Test
	def void checkErrorInitialValue() {
		"
			robot UnitTestRobot {
				component Component {
					behaviour {
						variable result_boolean: boolean = 0

						initial marked state idle {}
					}
				}
			}
		".parse.assertError(
			RobotSupervisoryControllerDSLPackage.Literals.VARIABLE,
			TypeCheckRule.INVALID_TYPE
		)
	}
}
