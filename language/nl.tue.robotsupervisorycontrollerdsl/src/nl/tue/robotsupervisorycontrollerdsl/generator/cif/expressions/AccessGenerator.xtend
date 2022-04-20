package nl.tue.robotsupervisorycontrollerdsl.generator.cif.expressions

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LiteralValue
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumValue
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResultTransition
import javax.inject.Inject
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.PlantNames
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AccessibleItem
import java.util.List
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.naming.VariableNames

@Singleton
class AccessGenerator {
	@Inject extension PlantNames
	@Inject extension VariableNames

	def compilePath(Access access) {
		if (access.value !== null) return access.value.compileFirst(access)
		
		return access.items.get(0).compileFirst(access)
	}

	def dispatch compileFirst(LiteralValue entity, Access access) {
		val resultTransition = ModelHelper.findParentOfType(entity, ResultTransition)

		if (resultTransition !== null) {
			val communicationType = resultTransition.communicationType

			return '''«communicationType.plantName».«resultTransition.resultType.inputName»«access.items.glueParts(true)»'''
		}
	}

	def dispatch compileFirst(Component entity, Access access) {
		if(access.items.size != 2) throw new UnsupportedOperationException("States cannot be accessed.")

		val list = newArrayList
		list.addAll(access.items)
		list.remove(0)

		return entity.accessName + '.' + list.glueParts(false)
	}

	def dispatch compileFirst(Variable entity, Access access) {
		if(access.items.size > 1) throw new UnsupportedOperationException("Variables cannot be accessed.")

		return entity.accessName
	}

	def dispatch compileFirst(EnumValue entity, Access access) {
		if(access.items.size > 1) throw new UnsupportedOperationException("Enums cannot be accessed.")

		return entity.accessName
	}

	private def glueParts(List<AccessibleItem> items, Boolean glueFirst) {
		val parts = items.map[it.accessName].filterNull

		if(parts.empty) return ''

		return (glueFirst ? '_' : '') + parts.join('_')
	}

	private dispatch def accessName(Component component) '''«component.name»'''

	private dispatch def accessName(EnumValue value) '''«value.name»'''

	private dispatch def accessName(Variable variable) '''«variable.variableName»'''

	private dispatch def accessName(State state) '''«state.name»'''

	private dispatch def accessName(LiteralValue variable) {
		return null
	}
}
