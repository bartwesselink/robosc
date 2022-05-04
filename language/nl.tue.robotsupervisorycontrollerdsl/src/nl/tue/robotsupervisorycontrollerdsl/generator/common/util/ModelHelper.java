package nl.tue.robotsupervisorycontrollerdsl.generator.common.util;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.EcoreUtil2;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ImportedComponent;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.LibraryDefinition;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;

public class ModelHelper {
	public static <T> T findParentOfType(EObject object, Class<T> type) {
		if (type.isInstance(object)) {
			return type.cast(object);
		} else {
			EObject container = object.eContainer();
			
			if (container != null) {
				return findParentOfType(container, type);
			}
			
			return null;
		}
	}

	public static <T extends EObject> List<T> findChildren(EObject object, Class<T> type) {
		return EcoreUtil2.getAllContentsOfType(object, type);
	}
	
	public static <T extends EObject> List<T> findWithinRobot(Robot robot, Class<T> type) {
		List<T> result = new ArrayList<>();
		
		if (robot == null) {
			return result;
		}
		
		result.addAll(EcoreUtil2.getAllContentsOfType(robot, type));
		
		List<Component> components = EcoreUtil2.getAllContentsOfType(robot, Component.class); 
	
		for (Component c : components) {
			result.addAll(ModelHelper.findInComponent(c, type, false));
		}
		
		return result;
	}
	
	public static <T extends EObject> List<T> findInComponent(Component component, Class<T> type) {
		return findInComponent(component, type, true);
	}
	
	public static <T extends EObject> List<T> findInComponent(Component component, Class<T> type, boolean includeDirectComponentChildren) {
		List<T> result = new ArrayList<>();
		
		if (component == null) {
			return result;
		}
		
		if (includeDirectComponentChildren) {
			result.addAll(EcoreUtil2.getAllContentsOfType(component, type));
		}
		
		Component current = component;
		
		while (current.getType() instanceof ImportedComponent) {
			LibraryDefinition definition = ((ImportedComponent) current.getType()).getDefinition();
			
			if (definition instanceof Component) {
				current = (Component) definition;
				result.addAll(EcoreUtil2.getAllContentsOfType(current, type));
			} else if (definition == null || definition.eContainer() == null) {
				break;
			}
		}
		
		return result;
	}
}
