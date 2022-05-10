package nl.tue.robotsupervisorycontrollerdsl.generator.common.data;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.inject.Inject;
import javax.inject.Singleton;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.EcoreUtil2;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AccessibleItem;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Assignment;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Expression;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LiteralValue;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Requirement;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResultTransition;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TransitionGuard;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable;

@Singleton
public class EliminationChecker {
	@Inject AccessHelper accessHelper;
	
	public boolean variableRequiredInSynthesis(Robot robot, Variable variable) {
		List<EObject> guards = ModelHelper.findWithinRobot(robot, TransitionGuard.class).stream()
				.map(it -> (EObject) it).collect(Collectors.toList());
		List<EObject> requirements = ModelHelper.findWithinRobot(robot, Requirement.class).stream()
				.map(it -> (EObject) it).collect(Collectors.toList());
		List<EObject> provideStatements = ModelHelper.findWithinRobot(robot, ProvideStatement.class).stream()
				.map(it -> (EObject) it.getExpression()).collect(Collectors.toList());

		if (listContainsVariableReference(guards, variable))
			return true;
		if (listContainsVariableReference(requirements, variable))
			return true;
		if (listContainsVariableReference(provideStatements, variable))
			return true;

		return false;
	}

	public boolean assignmentRequiredInSynthesis(Robot robot, Assignment assignment) {
		Variable variable = lastVariableOfAssignment(assignment);
	
		return variableRequiredInSynthesis(robot, variable);
	}
	
	public Variable lastVariableOfAssignment(Assignment assignment) {
		Access access = assignment.getItem();
		List<AccessibleItem> items = accessHelper.getAccessItems(access);
		
		AccessibleItem lastItem = items.get(items.size() - 1);
		
		if (!(lastItem instanceof Variable)) {
			throw new UnsupportedOperationException("Assignment must be a variable");
		}
		
		return (Variable) lastItem;
	}

	/**
	 * Function checks whether the incoming results of a communication type are used in a synthesis guard
	 * @param robot
	 * @param communication
	 * @return
	 */
	public boolean communicationTypeInputRequiredInSynthesis(Robot robot, CommunicationType communication) {
		List<ResultTransition> eventTransitions = ModelHelper.findWithinRobot(robot, ResultTransition.class);

		boolean value = eventTransitions.stream().filter(entity -> entity.getCommunicationType() == communication)
				.filter(entity -> entity.getAssignment() != null)
				.flatMap(entity -> entity.getAssignment().getAssignments().stream())
				.flatMap(entity -> findLiteralValues(entity))
				.map(entity -> ModelHelper.findParentOfType(entity, Access.class))
				.filter(entity -> accessHelper.getAccessItems(entity).isEmpty())
				.anyMatch(entity -> {
					Assignment assignment = ModelHelper.findParentOfType(entity, Assignment.class);
					Variable v = lastVariableOfAssignment(assignment);

					return variableRequiredInSynthesis(robot, v);
				});

		return value;

	}

	public boolean communicationTypeWithPropertyInputRequiredInSynthesis(Robot robot, CommunicationType communication,
			ObjectProperty p) {
		List<ResultTransition> eventTransitions = ModelHelper.findWithinRobot(robot, ResultTransition.class);

		boolean value = eventTransitions.stream().filter(entity -> entity.getCommunicationType() == communication)
				.filter(entity -> entity.getAssignment() != null)
				.flatMap(entity -> entity.getAssignment().getAssignments().stream())
				.flatMap(entity -> findLiteralValues(entity))
				.map(entity -> ModelHelper.findParentOfType(entity, Access.class))
				.filter(entity -> !accessHelper.getAccessItems(entity).isEmpty())
				.filter(entity -> accessPropertyIsSame(entity, p)).anyMatch(entity -> {
					Assignment assignment = ModelHelper.findParentOfType(entity, Assignment.class);
					Variable v = lastVariableOfAssignment(assignment);

					return variableRequiredInSynthesis(robot, v);
				});

		return value;

	}

	public boolean accessPropertyIsSame(Access access, ObjectProperty property) {
		List<AccessibleItem> items = accessHelper.getAccessItems(access);

		for (AccessibleItem item : items) {
			if (item == property)
				return true;
		}

		return false;
	}

	public Stream<LiteralValue> findLiteralValues(Expression e) {
		if (e == null) {
			return (new ArrayList<LiteralValue>()).stream();
		}

		if (e instanceof LiteralValue) {
			List<LiteralValue> list = new ArrayList<>();
			list.add((LiteralValue) e);

			return list.stream();
		}

		List<LiteralValue> candidates = EcoreUtil2.getAllContentsOfType(e, LiteralValue.class);
		return candidates.stream();
	}

	private boolean listContainsVariableReference(List<EObject> entities, Variable v) {
		return entities
				.stream()
				.filter(entity -> entity != null)
				.anyMatch(entity -> {
					List<Access> references = EcoreUtil2.getAllContentsOfType(entity, Access.class);
					
					if (references == null) {
						return false;
					}

					return references
							.stream()
							.anyMatch(it -> {
								return accessHelper.getAccessItems(it).contains(v);
							})
					;
							
				});
	}
}
