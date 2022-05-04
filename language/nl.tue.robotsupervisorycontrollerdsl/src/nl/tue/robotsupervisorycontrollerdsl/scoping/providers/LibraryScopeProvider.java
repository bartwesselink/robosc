package nl.tue.robotsupervisorycontrollerdsl.scoping.providers;

import javax.inject.Inject;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.scoping.IScope;
import org.eclipse.xtext.scoping.impl.DelegatingScopeProvider;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ImportedComponent;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.scoping.common.AbstractScopeProvider;

public class LibraryScopeProvider extends AbstractScopeProvider {
	@Inject DelegatingScopeProvider base;
	
	@Override
	public IScope getScope(EObject context, EReference reference) {	
		return base.getScope(context, reference);
	}

	@Override
	public boolean supports(EObject entity, EReference reference) {
		return entity instanceof ImportedComponent && reference == RobotSupervisoryControllerDSLPackage.Literals.IMPORTED_COMPONENT__LIBRARY;
	}
}