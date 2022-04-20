package nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.FeedbackResultType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResponseResultType

@Singleton
class VariableNames {
	def variableName(Variable variable)'''v_«variable.name»'''
	def dispatch inputName(FeedbackResultType type)'''i_feedback'''
	def dispatch inputName(ResponseResultType type)'''i_response'''
}