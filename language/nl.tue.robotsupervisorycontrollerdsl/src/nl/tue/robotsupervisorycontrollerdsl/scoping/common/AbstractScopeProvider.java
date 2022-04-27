package nl.tue.robotsupervisorycontrollerdsl.scoping.common;

import java.util.List;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;

public abstract class AbstractScopeProvider {
	public abstract List<EObject> determineCandidates(EObject context, EReference reference);
	public abstract boolean supports(EObject entity, EReference reference);
}
