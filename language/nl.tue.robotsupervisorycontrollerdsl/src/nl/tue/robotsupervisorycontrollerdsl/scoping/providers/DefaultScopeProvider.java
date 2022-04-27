package nl.tue.robotsupervisorycontrollerdsl.scoping.providers;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Singleton;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Library;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.scoping.common.AbstractScopeProvider;

@Singleton
public class DefaultScopeProvider extends AbstractScopeProvider {
	@SuppressWarnings("unchecked")
	@Override
	public List<EObject> determineCandidates(EObject context, EReference reference) {
		Robot robot = ModelHelper.findParentOfType(context, Robot.class);
		Library library = ModelHelper.findParentOfType(context, Library.class);

		if (robot != null) {
			return ModelHelper.findChildren(robot, (Class<EObject>) reference.getEReferenceType().getInstanceClass());
		} else if (library != null) {
			return ModelHelper.findChildren(library, (Class<EObject>) reference.getEReferenceType().getInstanceClass());
		}

		return new ArrayList<>();
	}

	@Override
	public boolean supports(EObject entity, EReference reference) {
		return true;
	}
}
