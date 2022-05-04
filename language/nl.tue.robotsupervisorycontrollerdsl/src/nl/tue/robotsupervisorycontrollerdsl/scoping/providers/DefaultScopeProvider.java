package nl.tue.robotsupervisorycontrollerdsl.scoping.providers;

import java.util.ArrayList;

import javax.inject.Singleton;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.scoping.IScope;
import org.eclipse.xtext.scoping.Scopes;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Library;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.scoping.common.AbstractScopeProvider;

@Singleton
public class DefaultScopeProvider extends AbstractScopeProvider {
	@SuppressWarnings("unchecked")
	@Override
	public IScope getScope(EObject context, EReference reference) {
		Robot robot = ModelHelper.findParentOfType(context, Robot.class);
		Library library = ModelHelper.findParentOfType(context, Library.class);

		if (robot != null) {
			return Scopes.scopeFor(ModelHelper.findChildren(robot, (Class<EObject>) reference.getEReferenceType().getInstanceClass()));
		} else if (library != null) {
			return Scopes.scopeFor(ModelHelper.findChildren(library, (Class<EObject>) reference.getEReferenceType().getInstanceClass()));
		}

		return Scopes.scopeFor(new ArrayList<>());
	}

	@Override
	public boolean supports(EObject entity, EReference reference) {
		return true;
	}
}
