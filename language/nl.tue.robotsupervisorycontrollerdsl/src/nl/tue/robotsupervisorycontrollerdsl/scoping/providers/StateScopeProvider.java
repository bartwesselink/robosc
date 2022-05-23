package nl.tue.robotsupervisorycontrollerdsl.scoping.providers;

import javax.inject.Singleton;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.scoping.IScope;
import org.eclipse.xtext.scoping.Scopes;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Automaton;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TransitionStateChange;
import nl.tue.robotsupervisorycontrollerdsl.scoping.common.AbstractScopeProvider;

@Singleton
public class StateScopeProvider extends AbstractScopeProvider {
	@Override
	public IScope getScope(EObject context, EReference reference) {
		Automaton automaton = ModelHelper.findParentOfType(context, Automaton.class);
		
		return Scopes.scopeFor(ModelHelper.findChildren(automaton, State.class));
	}

	@Override
	public boolean supports(EObject entity, EReference reference) {
		return entity instanceof TransitionStateChange
				&& reference == RobotSupervisoryControllerDSLPackage.Literals.TRANSITION_STATE_CHANGE__STATE;
	}
}
