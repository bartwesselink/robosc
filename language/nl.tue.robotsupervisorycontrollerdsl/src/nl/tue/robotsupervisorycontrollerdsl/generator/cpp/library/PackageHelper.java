package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.library;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javax.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.ros.AbstractPlatformTypeGenerator;
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Interface;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service;

@Singleton
public class PackageHelper {
	public List<String> getRequiredPackages(Robot robot) {
		List<String> result = new ArrayList<>();
	
		List<Interface> settings = ModelHelper.findWithinRobot(robot, CommunicationType.class)
				.stream()
				.map(it -> it.getLinks())
				.filter(it -> it != null)
				.collect(Collectors.toList());
		
		for (Interface setting : settings) {
			if (!result.contains(setting.getInterfacePackage())) {
				result.add(setting.getInterfacePackage());
			}
		}
	
		return result;
	}

	public List<CharSequence> getAllImports(Robot robot, AbstractPlatformTypeGenerator typeGenerator) {
		List<CharSequence> result = new ArrayList<>();
		
		// First, we list all imports from communication types
		List<CommunicationType> communicationTypes = ModelHelper.findWithinRobot(robot, CommunicationType.class);
		result.addAll(getImportsForCommunicationTypes(communicationTypes, typeGenerator));

		// Now, for each enum, we check what communication type it was associated with
		List<EnumDataType> enums = ModelHelper.findWithinRobot(robot, EnumDataType.class);
		result.addAll(enums
				.stream()
				
				// Find all places where it was accessed
				.flatMap(it -> ModelHelper.findWithinRobot(robot, ComplexDataTypeReference.class)
						.stream()
						.filter(reference -> reference.getType() == it)
				)
				.map(it -> ModelHelper.findParentOfType(it, CommunicationType.class))
				.map(it -> getImportForCommunicationType(it, typeGenerator))
				.collect(Collectors.toList())
				
		);
		
		return result.stream().filter(it -> it != null).collect(Collectors.toList());
	}
	
	private List<CharSequence> getImportsForCommunicationTypes(List<CommunicationType> list, AbstractPlatformTypeGenerator typeGenerator) {
		List<CharSequence> result = new ArrayList<>();
		
		result.addAll(list.stream().map(it -> getImportForCommunicationType(it, typeGenerator)).collect(Collectors.toList()));
		
		return result;
	}
	
	private CharSequence getImportForCommunicationType(CommunicationType input, AbstractPlatformTypeGenerator typeGenerator) {		
		if (input == null || input.getLinks() == null) {
			return null;
		}
		
		if (input instanceof Message) {
			return typeGenerator.messageImport(input.getLinks());
		} else if (input instanceof Action) {
			return typeGenerator.actionImport(input.getLinks());
		} else if (input instanceof Service) {
			return typeGenerator.serviceImport(input.getLinks());
		}
		
		return null;
	}
}
