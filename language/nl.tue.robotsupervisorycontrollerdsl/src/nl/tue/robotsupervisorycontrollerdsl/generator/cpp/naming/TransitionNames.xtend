package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming

import javax.inject.Singleton
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TauTransition
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message

@Singleton
class TransitionNames {
	@Inject nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.TransitionNames cifTransitionNames
	@Inject PlantNames cifPlantNames

	def transitionName(TauTransition tauTransition)'''«tauTransition.parentPlantName»_«cifTransitionNames.transitionName(tauTransition)»_'''
	def transitionName(ProvideStatement statement)'''«statement.parentPlantName»_«cifTransitionNames.transitionName(statement)»_'''
	
	def cancelTransitionName(CommunicationType communicationType)'''«cifPlantNames.plantName(communicationType)»_«cifTransitionNames.cancelTransitionName(communicationType)»_'''
	def triggerTransitionName(CommunicationType communicationType)'''«cifPlantNames.plantName(communicationType)»_«cifTransitionNames.triggerTransitionName(communicationType)»_'''
	def responseTransitionName(CommunicationType communicationType)'''«cifPlantNames.plantName(communicationType)»_«cifTransitionNames.responseTransitionName»_'''
	def resetTransitionName(CommunicationType communicationType)'''«cifPlantNames.plantName(communicationType)»_«cifTransitionNames.resetTransitionName(communicationType)»_'''
	def feedbackTransitionName(CommunicationType communicationType)'''«cifPlantNames.plantName(communicationType)»_«cifTransitionNames.feedbackTransitionName»_'''
	def errorTransitionName(CommunicationType communicationType)'''«cifPlantNames.plantName(communicationType)»_«cifTransitionNames.errorTransitionName»_'''
	def eventTransitionName(Message communicationType)'''«cifPlantNames.plantName(communicationType)»_«cifTransitionNames.transitionName(communicationType)»_'''
	
	def dispatch CharSequence parentPlantName(TauTransition transition) {
		val parent = ModelHelper.findParentOfType(transition, Component)
		
		return cifPlantNames.plantName(parent)
	}
	
	def dispatch CharSequence parentPlantName(ProvideStatement statement) {
		return cifPlantNames.dataPlantName(statement.communicationType)
	}
	
}

