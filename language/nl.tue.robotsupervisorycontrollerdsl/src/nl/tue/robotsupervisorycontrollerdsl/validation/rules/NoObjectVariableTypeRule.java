package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import org.eclipse.xtext.validation.Check;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class NoObjectVariableTypeRule extends AbstractValidationRule {
	public static final String NO_OBJECT_VARIABLE_TYPE = "noObjectVariableType";

	@Check
	public void checkVariableType(Variable entity) {
		if (entity.getType() instanceof ComplexDataTypeReference) {
			ComplexDataTypeReference reference = (ComplexDataTypeReference) entity.getType();
			ComplexDataType referenced = reference.getType();
			
			if (referenced instanceof ObjectDataType) {
				error("A variable can not have an object data type.",
						RobotSupervisoryControllerDSLPackage.Literals.VARIABLE__TYPE,
						NO_OBJECT_VARIABLE_TYPE);
			}
		}
	}
}
