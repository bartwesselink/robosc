package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class TypeSettingsRequiredRule extends AbstractValidationRule {
	public static final String TYPE_SETTINGS_REQUIRED = "typeSettingsRequired";

	@Check
	public void checkCommunicationTypeSettingsRequired(CommunicationType entity) {
		if (entity.getTypeSettings() != null) return;

		boolean required = false;
		
		if (entity instanceof Message) {
			required = referencesObjectDataType(((Message) entity).getType());
		} else if (entity instanceof Service) {
			required = referencesObjectDataType(((Service) entity).getResponseType())
					|| referencesObjectDataType(((Service) entity).getRequestType());
		} else if (entity instanceof Action) {
			required = referencesObjectDataType(((Action) entity).getResponseType())
					|| referencesObjectDataType(((Action) entity).getRequestType())
					|| referencesObjectDataType(((Action) entity).getFeedbackType());
		}
		
		if (required) {
			error("This communication type requires type settings.",
					RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE__TYPE_SETTINGS,
					TYPE_SETTINGS_REQUIRED);
		}
	}

	@Check
	public void checkEnumTypeSettingsRequired(EnumDataType entity) {
		if (entity.getTypeSettings() != null) return;
		
		if (referencesObjectDataType(entity.getType())) {
			error("This enum data type requires type settings.",
					RobotSupervisoryControllerDSLPackage.Literals.ENUM_DATA_TYPE__TYPE_SETTINGS,
					TYPE_SETTINGS_REQUIRED);
		}
	}
	
	private boolean referencesObjectDataType(DataType entity) {
		if (entity instanceof ComplexDataTypeReference) {
			ComplexDataTypeReference reference = (ComplexDataTypeReference) entity;
			
			return reference.getType() instanceof ObjectDataType;
		}
		
		return false;
	}
}
