package nl.tue.robotsupervisorycontrollerdsl.scoping.providers;

import java.util.stream.Collectors;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.scoping.IScope;
import org.eclipse.xtext.scoping.Scopes;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ImportedComponent;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Library;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.scoping.common.AbstractScopeProvider;

public class LibraryDefinitionScopeProvider extends AbstractScopeProvider {
	@Override
	public IScope getScope(EObject context, EReference reference) {
		Library library = ((ImportedComponent) context).getLibrary();

		return Scopes.scopeFor(ModelHelper.findChildren(library, Component.class)
				.stream()
				.map(it -> (EObject) it)
				.collect(Collectors.toList()));
	}

	@Override
	public boolean supports(EObject entity, EReference reference) {
		return entity instanceof ImportedComponent && reference == RobotSupervisoryControllerDSLPackage.Literals.IMPORTED_COMPONENT__DEFINITION;
	}
}