package nl.tue.robotsupervisorycontrollerdsl.scoping.common;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.scoping.IScope;

public abstract class AbstractScopeProvider {
	public abstract IScope getScope(EObject context, EReference reference);
	public abstract boolean supports(EObject entity, EReference reference);
}
