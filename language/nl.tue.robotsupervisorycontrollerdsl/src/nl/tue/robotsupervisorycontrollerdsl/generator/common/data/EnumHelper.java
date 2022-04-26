package nl.tue.robotsupervisorycontrollerdsl.generator.common.data;

import javax.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDefaultRule;

@Singleton
public class EnumHelper {	
	public EnumDefaultRule defaultRule(EnumDataType dataType) {
		return dataType
				.getRules()
				.stream()
				.filter(it -> it instanceof EnumDefaultRule)
				.map(it -> (EnumDefaultRule) it)
				.findFirst()
				.orElse(null);
	}
}
