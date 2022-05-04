package nl.tue.robotsupervisorycontrollerdsl.scoping.providers;

import java.util.stream.Collectors;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.scoping.IScope;
import org.eclipse.xtext.scoping.Scopes;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationTypeSet;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationTypeSingle;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Requirement;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.scoping.common.AbstractScopeProvider;

public class CommunicationTypeScopeProvider extends AbstractScopeProvider {
	@Override
	public IScope getScope(EObject context, EReference reference) {
		Robot robot = ModelHelper.findParentOfType(context, Robot.class);
		
		return Scopes.scopeFor(ModelHelper.findWithinRobot(robot, CommunicationType.class)
				.stream()
				.map(it -> (EObject) it)
				.collect(Collectors.toList()));
	}

	@Override
	public boolean supports(EObject entity, EReference reference) {
		return (entity instanceof Requirement && reference == RobotSupervisoryControllerDSLPackage.Literals.REQUIREMENT__COMMUNICATION_TYPE)
				|| (entity instanceof ProvideStatement && reference == RobotSupervisoryControllerDSLPackage.Literals.PROVIDE_STATEMENT__COMMUNICATION_TYPE)
				|| (entity instanceof CommunicationTypeSet && reference == RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE_SET__COMMUNICATION_TYPES)
				|| (entity instanceof CommunicationTypeSingle && reference == RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE_SINGLE__COMMUNICATION_TYPE);
	}

}