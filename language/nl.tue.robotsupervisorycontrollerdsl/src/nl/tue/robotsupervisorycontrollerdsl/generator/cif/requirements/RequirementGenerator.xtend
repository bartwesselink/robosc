package nl.tue.robotsupervisorycontrollerdsl.generator.cif.requirements

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Requirement
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.expressions.ExpressionGenerator
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationTypeSingle
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationTypeSet
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.TransitionNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot

@Singleton
class RequirementGenerator {
	@Inject extension ExpressionGenerator
	@Inject extension PlantNames
	@Inject extension TransitionNames

	def compile(Requirement requirement) {
		if (requirement.needsExpression !== null) {
			return requirement.compileNeeds
		} else {
			return requirement.compileDisables
		}
	}
	
	private def compileNeeds(Requirement requirement)'''
		«FOR communicationType : requirement.communicationTypes»
		requirement «communicationType.plantName».«communicationType.triggerTransitionName» needs «requirement.needsExpression.compile»;
		«ENDFOR»
	'''
	
	def compileCancelActionInverse(Action action, Robot robot) {
		// Enable cancel if any of the conditions do not holds anymore when the action was already started
		val disjunction = newArrayList()
		
		val actionRequirements = ModelHelper.findWithinRobot(robot, Requirement)
			.filter[(it.communicationType instanceof CommunicationTypeSingle && (it.communicationType as CommunicationTypeSingle).communicationType == action)
				|| (it.communicationType instanceof CommunicationTypeSet && (it.communicationType as CommunicationTypeSet).communicationTypes.contains(action))
			]
			
		for (requirement : actionRequirements) {
			if (requirement.needsExpression !== null) {
				disjunction.add('''(not («requirement.needsExpression.compile»))''')
			} else if (requirement.disablesExpression !== null) {
				disjunction.add('''(«requirement.needsExpression.compile»)''')
			}
		}
		
		if (!disjunction.empty) {
			return '''requirement «action.plantName».«action.cancelTransitionName» needs «disjunction.join(' or ')»;'''
		}
	}
	
	private def compileDisables(Requirement requirement)'''
		«FOR communicationType : requirement.communicationTypes»
		requirement «requirement.disablesExpression.compile» disables «communicationType.plantName».«communicationType.triggerTransitionName»;
				
		«IF communicationType instanceof Action»
		// Enable cancel when the action was already started
		requirement not («requirement.disablesExpression.compile») disables «communicationType.plantName».«communicationType.cancelTransitionName»;
		«ENDIF»		
		«ENDFOR»
	'''
	
	private def communicationTypes(Requirement requirement) {
		val type = requirement.communicationType
		
    	if (type instanceof CommunicationTypeSingle) {
    		return newArrayList(type.communicationType)
    	} else if (type instanceof CommunicationTypeSet) {
    		val types = newArrayList()
    		types.addAll(type.communicationTypes)
    		
    		return types
    	} else {
    		return newArrayList
    	}
    }
}