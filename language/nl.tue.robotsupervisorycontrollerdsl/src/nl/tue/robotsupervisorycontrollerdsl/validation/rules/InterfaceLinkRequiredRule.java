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

public class InterfaceLinkRequiredRule extends AbstractValidationRule {
	public static final String TYPE_SETTINGS_REQUIRED = "typeSettingsRequired";

	@Check
	public void checkCommunicationTypeInterfaceLinkRequired(CommunicationType entity) {
		if (entity.getLinks() != null) return;
		
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
					RobotSupervisoryControllerDSLPackage.Literals.COMMUNICATION_TYPE__LINKS,
					TYPE_SETTINGS_REQUIRED);
		}
	}

	private boolean referencesObjectDataType(DataType entity) {
		if (entity instanceof ComplexDataTypeReference) {
			ComplexDataTypeReference reference = (ComplexDataTypeReference) entity;
			
			if (reference.getType() instanceof ObjectDataType) {
				return true;
			} else if (reference.getType() instanceof EnumDataType) {
				EnumDataType enumType = (EnumDataType) reference.getType();
				
				return enumType.getType() instanceof ComplexDataTypeReference;
			}
			
			return false;
		}
		
		return false;
	}
}
