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
		
		«IF communicationType instanceof Action»
		// Enable cancel when the action was already started
		requirement «communicationType.plantName».«communicationType.cancelTransitionName» needs not («requirement.needsExpression.compile»);
		«ENDIF»		
		«ENDFOR»
	'''
	
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