package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.expressions

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Expression
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.And
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Or
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Inequality
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.GreaterThan
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Equation
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.GreaterThanEqual
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Implies
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Plus
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Minus
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Multiply
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Divide
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Negation
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Negative
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.SmallerThanEqual
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.SmallerThan
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Parenthesized
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LiteralDouble
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LiteralInt
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LiteralString
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LiteralBoolean
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Atom
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Literal
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Assignment

@Singleton
class ExpressionGenerator {
	@Inject extension AccessGenerator
	
	def compile(Expression expression)'''«expression.compileValue(null)»'''
	def compile(Literal literal)'''«literal.compileValue(null)»'''
	
	def compile(Expression expression, String literalValue)'''«expression.compileValue(literalValue)»'''
	def compile(Literal literal, String literalValue)'''«literal.compileValue(literalValue)»'''

	private def dispatch String compileValue(And value, String literalValue) '''«value.left.compileValue(literalValue)» && «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(Or value, String literalValue) '''«value.left.compileValue(literalValue)» || «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(Equation value, String literalValue) '''«value.left.compileValue(literalValue)» == «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(Inequality value, String literalValue) '''«value.left.compileValue(literalValue)» != «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(GreaterThan value, String literalValue) '''«value.left.compileValue(literalValue)» > «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(GreaterThanEqual value, String literalValue) '''«value.left.compileValue(literalValue)» >= «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(SmallerThan value, String literalValue) '''«value.left.compileValue(literalValue)» < «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(SmallerThanEqual value, String literalValue) '''«value.left.compileValue(literalValue)» <= «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(Implies value, String literalValue) '''!(«value.left.compileValue(literalValue)») || («value.right.compileValue(literalValue)»)'''
	private def dispatch String compileValue(Plus value, String literalValue) '''«value.left.compileValue(literalValue)» + «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(Minus value, String literalValue) '''«value.left.compileValue(literalValue)» - «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(Multiply value, String literalValue) '''«value.left.compileValue(literalValue)» * «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(Divide value, String literalValue) '''«value.left.compileValue(literalValue)» / «value.right.compileValue(literalValue)»'''
	private def dispatch String compileValue(Negation value, String literalValue) '''not «value.value.compileValue(literalValue)»'''
	private def dispatch String compileValue(Negative value, String literalValue) '''-«value.value.compileValue(literalValue)»'''
	private def dispatch String compileValue(Parenthesized value, String literalValue) '''(«value.value.compileValue(literalValue)»)'''
	private def dispatch String compileValue(Atom value, String literalValue) '''«value.value.compileValue(literalValue)»'''
	private def dispatch String compileValue(Access access, String literalValue) '''«access.compilePath(literalValue)»'''
	private def dispatch String compileValue(Assignment value, String literalValue) '''«value.item.compilePath()» = «value.value.compileValue(literalValue)»'''

	private def dispatch String compileValue(LiteralDouble value, String literalValue) '''«value.one».«value.two»'''
	private def dispatch String compileValue(LiteralInt value, String literalValue) '''«value.value»'''
	private def dispatch String compileValue(LiteralString value, String literalValue) '''"«value.value»"'''
	private def dispatch String compileValue(LiteralBoolean value, String literalValue) '''«value.value»'''
}