package nl.tue.robotsupervisorycontrollerdsl.generator.common.data;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AccessType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AccessibleItem;

@Singleton
public class AccessHelper {
	public List<AccessibleItem> getAccessItems(Access entity) {
		List<AccessibleItem> result = new ArrayList<>();
		
		if (entity.getFirstItem() != null) result.add(entity.getFirstItem());
		
		for (AccessType type : entity.getTypes()) {
			if (type.getItem() != null) result.add(type.getItem());
		}
		
		return result;
	}
}
