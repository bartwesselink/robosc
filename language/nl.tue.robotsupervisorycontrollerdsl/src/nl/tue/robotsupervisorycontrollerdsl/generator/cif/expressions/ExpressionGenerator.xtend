package nl.tue.robotsupervisorycontrollerdsl.generator.cif.expressions

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
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Negation
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Negative

@Singleton
class ExpressionGenerator {
	@Inject extension AccessGenerator
	
	def compile(Expression expression)'''«expression.compileValue»'''
	def compile(Literal literal)'''«literal.compileValue»'''

	private def dispatch String compileValue(And value) '''«value.left.compileValue» and «value.right.compileValue»'''
	private def dispatch String compileValue(Or value) '''«value.left.compileValue» or «value.right.compileValue»'''
	private def dispatch String compileValue(Equation value) '''«value.left.compileValue» = «value.right.compileValue»'''
	private def dispatch String compileValue(Inequality value) '''«value.left.compileValue» != «value.right.compileValue»'''
	private def dispatch String compileValue(GreaterThan value) '''«value.left.compileValue» > «value.right.compileValue»'''
	private def dispatch String compileValue(GreaterThanEqual value) '''«value.left.compileValue» >= «value.right.compileValue»'''
	private def dispatch String compileValue(SmallerThan value) '''«value.left.compileValue» < «value.right.compileValue»'''
	private def dispatch String compileValue(SmallerThanEqual value) '''«value.left.compileValue» <= «value.right.compileValue»'''
	private def dispatch String compileValue(Implies value) '''«value.left.compileValue» => «value.right.compileValue»'''
	private def dispatch String compileValue(Plus value) '''«value.left.compileValue» + «value.right.compileValue»'''
	private def dispatch String compileValue(Minus value) '''«value.left.compileValue» - «value.right.compileValue»'''
	private def dispatch String compileValue(Multiply value) '''«value.left.compileValue» * «value.right.compileValue»'''
	private def dispatch String compileValue(Divide value) '''«value.left.compileValue» / «value.right.compileValue»'''
	private def dispatch String compileValue(Negation value) '''not «value.value.compileValue»'''
	private def dispatch String compileValue(Negative value) '''-«value.value.compileValue»'''
	private def dispatch String compileValue(Parenthesized value) '''(«value.value.compileValue»)'''
	private def dispatch String compileValue(Atom value) '''«value.value.compileValue»'''
	private def dispatch String compileValue(Access access) '''«access.compilePath»'''
	private def dispatch String compileValue(Assignment value) '''«value.item.compilePath» := «value.value.compileValue»'''

	private def dispatch String compileValue(LiteralDouble value) '''«value.one».«value.two»'''
	private def dispatch String compileValue(LiteralInt value) '''«value.value»'''
	private def dispatch String compileValue(LiteralString value) '''"«value.value»"'''
	private def dispatch String compileValue(LiteralBoolean value) '''«value.value»'''
}