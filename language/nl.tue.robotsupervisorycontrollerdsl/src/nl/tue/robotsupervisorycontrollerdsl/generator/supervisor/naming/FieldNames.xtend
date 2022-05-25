package nl.tue.robotsupervisorycontrollerdsl.generator.supervisor.naming

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action

class FieldNames extends nl.tue.robotsupervisorycontrollerdsl.generator.cpp.naming.FieldNames {
	def fieldNameSupervised(CommunicationType entity) '''supervised_«entity.fieldName»'''
	def mutexLockNameSupervised(CommunicationType entity) '''mutex_supervised_«entity.fieldName»'''
	def dataHolderNameSupervised(CommunicationType entity) '''data_holder_supervised_«entity.fieldName»'''
	def responseHolderNameSupervised(CommunicationType entity) '''response_holder_supervised_«entity.fieldName»'''
	def conditionVariableSupervised(CommunicationType entity) '''condition_variable_supervised_«entity.fieldName»'''
	def responseReadySupervised(CommunicationType entity) '''response_ready_«entity.fieldName»'''
	def goalHandleNameSupervised(Action entity) '''goal_handle_supervised_«entity.fieldName»'''
}