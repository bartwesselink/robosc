package nl.tue.robotsupervisorycontrollerdsl.tests.typesystem

import com.google.inject.Inject
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith
import nl.tue.robotsupervisorycontrollerdsl.tests.RobotSupervisoryControllerDSLInjectorProvider
import nl.tue.robotsupervisorycontrollerdsl.typesystem.ExpressionTypesystem
import nl.tue.robotsupervisorycontrollerdsl.typesystem.TypesystemDataType
import org.junit.jupiter.api.Assertions
import nl.tue.robotsupervisorycontrollerdsl.tests.helper.ParseHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access
import org.eclipse.xtext.EcoreUtil2

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class ExpressionTypesystemTest {
	@Inject ParseHelper parseHelper
	@Inject ExpressionTypesystem expressionTypesystem

	@Test
	def void checkBooleanOperators() {
		"true or false".assertExpressionType(TypesystemDataType.BOOLEAN)
		"true and false".assertExpressionType(TypesystemDataType.BOOLEAN)
		"true => false".assertExpressionType(TypesystemDataType.BOOLEAN)
		"5 > 2".assertExpressionType(TypesystemDataType.BOOLEAN)
		"5 >= 2".assertExpressionType(TypesystemDataType.BOOLEAN)
		"5 < 2".assertExpressionType(TypesystemDataType.BOOLEAN)
		"5 <= 2".assertExpressionType(TypesystemDataType.BOOLEAN)
		"!true".assertExpressionType(TypesystemDataType.BOOLEAN)
	}

	@Test
	def void checkDataTypes() {
		"double".assertDataTypeType(TypesystemDataType.DOUBLE)
		"none".assertDataTypeType(TypesystemDataType.NONE)
		"boolean".assertDataTypeType(TypesystemDataType.BOOLEAN)
		"integer".assertDataTypeType(TypesystemDataType.INT)
		"string".assertDataTypeType(TypesystemDataType.STRING)
	}

	@Test
	def void checkLiterals() {
		"5.0".assertExpressionType(TypesystemDataType.DOUBLE)
		"true".assertExpressionType(TypesystemDataType.BOOLEAN)
		"1".assertExpressionType(TypesystemDataType.INT)
		"\"test\"".assertExpressionType(TypesystemDataType.STRING)
	}

	@Test
	def void checkVariables() {
		"test: integer(0..20) = 0".assertVariableType(TypesystemDataType.INT)
		"test: boolean".assertVariableType(TypesystemDataType.BOOLEAN)
	}

	@Test
	def void checkNumericOperators() {
		"5 + 2".assertExpressionType(TypesystemDataType.INT)
		"5.0 + 1".assertExpressionType(TypesystemDataType.DOUBLE)
		"2 * 3".assertExpressionType(TypesystemDataType.INT)
		"3.0 / 2.0".assertExpressionType(TypesystemDataType.DOUBLE)
		"1 - 4".assertExpressionType(TypesystemDataType.INT)
		"(1 - 4)".assertExpressionType(TypesystemDataType.INT)
	}

	@Test
	def void checkValueAccessEnums() {
		"
			robot UnitTest {
				datatype enum Test from integer to {
					(value > 5) -> yes
					default -> no
				}
			}
		".assertValueAccessType(TypesystemDataType.INT)
		
		
		"
			robot UnitTest {
				datatype object ComplexObject {
					property: array(integer)
				}

				datatype enum Test from ComplexObject to {
					(value.property[0] > 5) -> yes
					default -> no
				}
			}
		".assertValueAccessType(TypesystemDataType.INT)
	}

	@Test
	def void checkValueAccessResultTransitions() {
		"
			robot UnitTest {
				component One {
					incoming message test_message with type: integer

					behaviour {
						variable current: integer

						initial state idle {
							on response from test_message do current := value
						}
					}
				}
			}
		".assertValueAccessType(TypesystemDataType.INT)
		
		
		"
			robot UnitTest {
				datatype object Complex {
					outcome: boolean
				}

				component One {
					incoming message test_message with type: Complex

					behaviour {
						variable current: boolean

						initial state idle {
							on response from test_message do current := value.outcome
						}
					}
				}
			}
		".assertValueAccessType(TypesystemDataType.BOOLEAN)
	}

	@Test
	def void checkStateAccess() {
		"
			robot UnitTest {
				component One {
					outgoing message test_message with type: integer

					behaviour {
						initial state idle {}
					}
				}

				requirement test_message needs One.idle
			}
		".assertItemAccessType(TypesystemDataType.BOOLEAN)
	}

	@Test
	def void checkComponentVariableAccess() {
		"
			robot UnitTest {
				component One {
					behaviour {
						variable test_variable: boolean = false

						initial state idle {}
					}
				}

				requirement test_message needs One.test_variable
			}
		".assertItemAccessType(TypesystemDataType.BOOLEAN)
	}
	
	private def assertExpressionType(String input, TypesystemDataType<?> type) {
		val parsed = parseHelper.parseExpression(input)

		Assertions.assertEquals(type, expressionTypesystem.typeOf(parsed))
	}
	
	private def assertDataTypeType(String input, TypesystemDataType<?> type) {
		val parsed = parseHelper.parseDataType(input)

		Assertions.assertEquals(type, expressionTypesystem.typeOf(parsed))
	}
	
	private def assertVariableType(String input, TypesystemDataType<?> type) {
		val parsed = parseHelper.parseVariable(input)

		Assertions.assertEquals(type, expressionTypesystem.typeOf(parsed))
	}
	
	private def assertValueAccessType(String input, TypesystemDataType<?> type) {
		val parsed = parseHelper.parseBase(input)
		val allAccess = EcoreUtil2.getAllContentsOfType(parsed, Access).filter[it.value !== null]
		val access = !allAccess.empty ? allAccess.get(0) : null

		Assertions.assertEquals(type, expressionTypesystem.typeOf(access))
	}
	
	private def assertItemAccessType(String input, TypesystemDataType<?> type) {
		val parsed = parseHelper.parseBase(input)
		val allAccess = EcoreUtil2.getAllContentsOfType(parsed, Access).filter[it.value === null]
		val access = !allAccess.empty ? allAccess.get(0) : null

		Assertions.assertEquals(type, expressionTypesystem.typeOf(access))
	}
}
