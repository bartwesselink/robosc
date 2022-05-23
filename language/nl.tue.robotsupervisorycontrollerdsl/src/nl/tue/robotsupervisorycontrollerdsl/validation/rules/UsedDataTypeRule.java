package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import javax.inject.Inject;

import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.data.EliminationChecker;
import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ArrayDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DoubleDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.StringDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class UsedDataTypeRule extends AbstractValidationRule {
	@Inject EliminationChecker eliminationChecker;
	
	public static final String UNSUPPORTED_VARIABLE_TYPE = "unsupportedVariableType";
	public static final String NO_ARRAY_ALLOWED = "noArrayAllowed";
	public static final String NO_DOUBLE_ALLOWED = "noDoubleAllowed";
	public static final String NO_STRING_ALLOWED = "noStringAllowed";
	public static final String NO_ENUM_ALLOWED = "noEnumAllowed";

	@Check
	public void checkVariableType(Variable entity) {
		if (entity.getType() instanceof ComplexDataTypeReference) {
			ComplexDataTypeReference reference = (ComplexDataTypeReference) entity.getType();
			ComplexDataType referenced = reference.getType();
			
			if (referenced instanceof ObjectDataType) {
				error("This variable type is unsupported.",
						RobotSupervisoryControllerDSLPackage.Literals.VARIABLE__TYPE,
						UNSUPPORTED_VARIABLE_TYPE);
			}
		} else if (entity.getType() instanceof ArrayDataType) {
			error("Arrays are not supported for this variable as it is being used in the controller.",
					RobotSupervisoryControllerDSLPackage.Literals.VARIABLE__TYPE,
					NO_ARRAY_ALLOWED);
		} else {
			Robot robot = ModelHelper.findParentOfType(entity, Robot.class);

			if (robot == null || this.eliminationChecker.variableRequiredInSynthesis(robot, entity)) {
				if (entity.getType() instanceof DoubleDataType) {
					error("Doubles are not supported for this variable as it is being used in the controller. You can use enums.",
							RobotSupervisoryControllerDSLPackage.Literals.VARIABLE__TYPE,
							NO_DOUBLE_ALLOWED);
				} else if (entity.getType() instanceof StringDataType) {
					error("Strings are not supported for this variable as it is being used in the controller. You can use enums.",
							RobotSupervisoryControllerDSLPackage.Literals.VARIABLE__TYPE,
							NO_STRING_ALLOWED);
				}
			}
		}
	}
	
	@Check
	public void checkEnumType(EnumDataType entity) {
		if (entity.getType() instanceof ComplexDataTypeReference) {
			ComplexDataType referenced = ((ComplexDataTypeReference) entity.getType()).getType();
			
			if (referenced instanceof EnumDataType) {
				error("It is not allowed to convert from enums to enums.",
						RobotSupervisoryControllerDSLPackage.Literals.ENUM_DATA_TYPE__TYPE,
						NO_ENUM_ALLOWED);
			}
		}
	}
	
	@Check
	public void checkCommunicationDataType(CommunicationType entity) {
		if (entity instanceof Message) {
			checkArrayDataType(((Message) entity).getType(), RobotSupervisoryControllerDSLPackage.Literals.MESSAGE__TYPE);
		} else if (entity instanceof Service) {
			checkArrayDataType(((Service) entity).getRequestType(), RobotSupervisoryControllerDSLPackage.Literals.SERVICE__REQUEST_TYPE);
			checkArrayDataType(((Service) entity).getResponseType(), RobotSupervisoryControllerDSLPackage.Literals.SERVICE__RESPONSE_TYPE);
		} else if (entity instanceof Action) {
			checkArrayDataType(((Action) entity).getRequestType(), RobotSupervisoryControllerDSLPackage.Literals.ACTION__REQUEST_TYPE);
			checkArrayDataType(((Action) entity).getResponseType(), RobotSupervisoryControllerDSLPackage.Literals.ACTION__RESPONSE_TYPE);
			checkArrayDataType(((Action) entity).getFeedbackType(), RobotSupervisoryControllerDSLPackage.Literals.ACTION__FEEDBACK_TYPE);
		}
	}
	
	private void checkArrayDataType(DataType type, EReference reference) {
		if (type instanceof ArrayDataType) {
			error("Arrays can not be used here.",
					reference,
					NO_ARRAY_ALLOWED);
		}
	}
}
