package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.library;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javax.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.generator.ros2.data.PlatformTypeGenerator;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CustomTypeSettings;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service;

@Singleton
public class PackageHelper {
	public List<String> getRequiredPackages(Robot robot) {
		return ModelHelper.findWithinRobot(robot, CustomTypeSettings.class)
				.stream()
				.map(it -> it.getPackage())
				.collect(Collectors.toList());
	}
	
	public List<String> getAllImports(Robot robot, PlatformTypeGenerator typeGenerator) {
		List<String> result = new ArrayList<>();
		
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
	
	private List<String> getImportsForCommunicationTypes(List<CommunicationType> list, PlatformTypeGenerator typeGenerator) {
		List<String> result = new ArrayList<>();
		
		result.addAll(list.stream().map(it -> getImportForCommunicationType(it, typeGenerator)).collect(Collectors.toList()));
		
		return result;
	}
	
	private String getImportForCommunicationType(CommunicationType input, PlatformTypeGenerator typeGenerator) {		
		if (input == null || input.getTypeSettings() == null) {
			return null;
		}
		
		if (input instanceof Message) {
			return typeGenerator.messageImport(input.getTypeSettings());
		} else if (input instanceof Action) {
			return typeGenerator.actionImport(input.getTypeSettings());
		} else if (input instanceof Service) {
			return typeGenerator.serviceImport(input.getTypeSettings());
		}
		
		return null;
	}
}
