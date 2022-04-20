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
	
	public static <T extends EObject> List<T> findWithinRobot(Robot robot, Class<T> type) {
		List<T> result = new ArrayList<>();
		
		if (robot == null) {
			return result;
		}
		
		result.addAll(EcoreUtil2.getAllContentsOfType(robot, type));
		
		List<Component> components = EcoreUtil2.getAllContentsOfType(robot, Component.class); 
		
		for (Component c : components) {
			Component current = c;
			
			while (current.getType() instanceof ImportedComponent) {
				LibraryDefinition definition = ((ImportedComponent) current.getType()).getDefinition();
				
				if (current instanceof Component) {
					current = (Component) definition;
					result.addAll(EcoreUtil2.getAllContentsOfType(current, type));
				}
			}
		}
		
		return result;
	}
}
