package nl.tue.robotsupervisorycontrollerdsl.scoping;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Inject;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.scoping.IScope;
import org.eclipse.xtext.scoping.Scopes;

import nl.tue.robotsupervisorycontrollerdsl.scoping.common.AbstractScopeProvider;
import nl.tue.robotsupervisorycontrollerdsl.scoping.providers.AccessScopeProvider;
import nl.tue.robotsupervisorycontrollerdsl.scoping.providers.CommunicationTypeScopeProvider;
import nl.tue.robotsupervisorycontrollerdsl.scoping.providers.DefaultScopeProvider;
import nl.tue.robotsupervisorycontrollerdsl.scoping.providers.LibraryDefinitionScopeProvider;
import nl.tue.robotsupervisorycontrollerdsl.scoping.providers.LibraryScopeProvider;
import nl.tue.robotsupervisorycontrollerdsl.scoping.providers.PropertyValueScopeProvider;

public class RobotSupervisoryControllerDSLScopeProvider extends AbstractRobotSupervisoryControllerDSLScopeProvider {
	@Inject DefaultScopeProvider defaultScopeProvider;
	@Inject CommunicationTypeScopeProvider communicationTypeScopeProvider;
	@Inject PropertyValueScopeProvider propertyValueScopeProvder;
	@Inject LibraryDefinitionScopeProvider libraryDefinitionScopeProvider;
	@Inject AccessScopeProvider accessScopeProvider;
	@Inject LibraryScopeProvider libraryScopeProvider;

	@Override
	public IScope getScope(EObject context, EReference reference) {
		List<AbstractScopeProvider> all = new ArrayList<>();
		all.add(propertyValueScopeProvder);
		all.add(libraryScopeProvider);
		all.add(communicationTypeScopeProvider);
		all.add(libraryDefinitionScopeProvider);
		all.add(accessScopeProvider);
		all.add(defaultScopeProvider);
		
		for (AbstractScopeProvider provider : all) {
			if (provider.supports(context, reference)) {
				return provider.getScope(context, reference); 
			}
		}

		return Scopes.scopeFor(new ArrayList<>());
	}
}
