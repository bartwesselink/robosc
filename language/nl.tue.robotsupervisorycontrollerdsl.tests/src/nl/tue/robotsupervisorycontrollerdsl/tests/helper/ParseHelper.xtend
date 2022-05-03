package nl.tue.robotsupervisorycontrollerdsl.tests.helper

import javax.inject.Inject
import org.eclipse.xtext.parser.IParser
import nl.tue.robotsupervisorycontrollerdsl.services.RobotSupervisoryControllerDSLGrammarAccess
import java.io.StringReader
import org.eclipse.xtext.parser.IParseResult
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Expression
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Base
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access

class ParseHelper {
	@Inject IParser parser
	@Inject org.eclipse.xtext.testing.util.ParseHelper<Base> defaultParser
	@Inject RobotSupervisoryControllerDSLGrammarAccess grammar
	
	def Expression parseExpression(CharSequence input) {
	    val IParseResult result = parser.parse(grammar.expressionRule, new StringReader(input.toString))
	    
	    return result.rootASTElement as Expression
	}

	def DataType parseDataType(CharSequence input) {
	    val IParseResult result = parser.parse(grammar.dataTypeRule, new StringReader(input.toString))
	    
	    return result.rootASTElement as DataType
	}

	def Base parseBase(CharSequence input) {
	    return defaultParser.parse(input)
	}

	def Variable parseVariable(CharSequence input) {
	    val IParseResult result = parser.parse(grammar.variableRule, new StringReader(input.toString))
	    
	    return result.rootASTElement as Variable
	}

	def Access parseAccess(CharSequence input) {
	    val IParseResult result = parser.parse(grammar.accessRule, new StringReader(input.toString))
	    
	    return result.rootASTElement as Access
	}
}