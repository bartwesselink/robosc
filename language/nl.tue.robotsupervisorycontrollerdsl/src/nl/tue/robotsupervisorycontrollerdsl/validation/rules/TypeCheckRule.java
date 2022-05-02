package nl.tue.robotsupervisorycontrollerdsl.validation.rules;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.validation.Check;

import com.google.inject.Inject;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.And;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Assignment;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Divide;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.GreaterThan;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.GreaterThanEqual;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Minus;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Multiply;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Negation;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectPropertyValue;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Or;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Plus;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.SmallerThan;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.SmallerThanEqual;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.TransitionGuard;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.WithDataValue;
import nl.tue.robotsupervisorycontrollerdsl.typesystem.ExpressionTypesystem;
import nl.tue.robotsupervisorycontrollerdsl.typesystem.TypesystemDataType;
import nl.tue.robotsupervisorycontrollerdsl.validation.common.AbstractValidationRule;

public class TypeCheckRule extends AbstractValidationRule {
	@Inject
	ExpressionTypesystem typesystem;

	public static final String INVALID_TYPE = "invalidType";

	@Check
	public void checkOr(Or entity) {
		ensureTypeCorrect(entity.getLeft(), TypesystemDataType.BOOLEAN,
				RobotSupervisoryControllerDSLPackage.Literals.OR__LEFT);
		ensureTypeCorrect(entity.getRight(), TypesystemDataType.BOOLEAN,
				RobotSupervisoryControllerDSLPackage.Literals.OR__RIGHT);
	}

	@Check
	public void checkAnd(And entity) {
		ensureTypeCorrect(entity.getLeft(), TypesystemDataType.BOOLEAN,
				RobotSupervisoryControllerDSLPackage.Literals.AND__LEFT);
		ensureTypeCorrect(entity.getRight(), TypesystemDataType.BOOLEAN,
				RobotSupervisoryControllerDSLPackage.Literals.AND__RIGHT);
	}

	@Check
	public void checkGreaterThan(GreaterThan entity) {
		ensureNumericTypeCorrect(entity.getLeft(), RobotSupervisoryControllerDSLPackage.Literals.GREATER_THAN__LEFT);
		ensureNumericTypeCorrect(entity.getRight(), RobotSupervisoryControllerDSLPackage.Literals.GREATER_THAN__RIGHT);
	}

	@Check
	public void checkSmallerThan(SmallerThan entity) {
		ensureNumericTypeCorrect(entity.getLeft(), RobotSupervisoryControllerDSLPackage.Literals.SMALLER_THAN__LEFT);
		ensureNumericTypeCorrect(entity.getRight(), RobotSupervisoryControllerDSLPackage.Literals.SMALLER_THAN__RIGHT);
	}

	@Check
	public void checkGreaterThanEqual(GreaterThanEqual entity) {
		ensureNumericTypeCorrect(entity.getLeft(), RobotSupervisoryControllerDSLPackage.Literals.GREATER_THAN_EQUAL__LEFT);
		ensureNumericTypeCorrect(entity.getRight(), RobotSupervisoryControllerDSLPackage.Literals.GREATER_THAN_EQUAL__RIGHT);
	}

	@Check
	public void checkSmallerThanEqual(SmallerThanEqual entity) {
		ensureNumericTypeCorrect(entity.getLeft(), RobotSupervisoryControllerDSLPackage.Literals.SMALLER_THAN_EQUAL__LEFT);
		ensureNumericTypeCorrect(entity.getRight(), RobotSupervisoryControllerDSLPackage.Literals.SMALLER_THAN_EQUAL__RIGHT);
	}

	@Check
	public void checkMultiply(Multiply entity) {
		ensureNumericTypeCorrect(entity.getLeft(), RobotSupervisoryControllerDSLPackage.Literals.MULTIPLY__LEFT);
		ensureNumericTypeCorrect(entity.getRight(), RobotSupervisoryControllerDSLPackage.Literals.MULTIPLY__RIGHT);
	}

	@Check
	public void checkDivide(Divide entity) {
		ensureNumericTypeCorrect(entity.getLeft(), RobotSupervisoryControllerDSLPackage.Literals.DIVIDE__LEFT);
		ensureNumericTypeCorrect(entity.getRight(), RobotSupervisoryControllerDSLPackage.Literals.DIVIDE__RIGHT);
	}

	@Check
	public void checkPlus(Plus entity) {
		ensureNumericTypeCorrect(entity.getLeft(), RobotSupervisoryControllerDSLPackage.Literals.PLUS__LEFT);
		ensureNumericTypeCorrect(entity.getRight(), RobotSupervisoryControllerDSLPackage.Literals.PLUS__RIGHT);
	}

	@Check
	public void checkMinus(Minus entity) {
		ensureNumericTypeCorrect(entity.getLeft(), RobotSupervisoryControllerDSLPackage.Literals.MINUS__LEFT);
		ensureNumericTypeCorrect(entity.getRight(), RobotSupervisoryControllerDSLPackage.Literals.MINUS__RIGHT);
	}

	@Check
	public void checkNegation(Negation entity) {
		ensureTypeCorrect(entity.getValue(), TypesystemDataType.BOOLEAN, RobotSupervisoryControllerDSLPackage.Literals.NEGATION__VALUE);
	}

	@Check
	public void checkTransitionGuardType(TransitionGuard entity) {
		ensureTypeCorrect(entity.getExpression(), TypesystemDataType.BOOLEAN, RobotSupervisoryControllerDSLPackage.Literals.TRANSITION_GUARD__EXPRESSION);
	}

	@Check
	public void checkProvidedDataType(ProvideStatement entity) {
		CommunicationType communication = entity.getCommunicationType();
		WithDataValue data = entity.getData() != null ? entity.getData().getData() : null;
		
		if (communication instanceof Message) {
			ensureTypeCorrect(data, typesystem.typeOf(((Message) communication).getType()), RobotSupervisoryControllerDSLPackage.Literals.PROVIDE_STATEMENT__DATA);
		} else if (communication instanceof Service) {
			ensureTypeCorrect(data, typesystem.typeOf(((Service) communication).getRequestType()), RobotSupervisoryControllerDSLPackage.Literals.PROVIDE_STATEMENT__DATA);
		} else if (communication instanceof Action) {
			ensureTypeCorrect(data, typesystem.typeOf(((Action) communication).getRequestType()), RobotSupervisoryControllerDSLPackage.Literals.PROVIDE_STATEMENT__DATA);
		}
	}

	@Check
	public void checkObjectValueType(ObjectPropertyValue entity) {
		ensureTypesEqual(entity.getValue(), entity.getProperty().getType(),
				RobotSupervisoryControllerDSLPackage.Literals.OBJECT_PROPERTY_VALUE__VALUE);
	}

	@Check
	public void checkAssignmentType(Assignment entity) {
		ensureTypesEqual(entity.getItem(), entity.getValue(),
				RobotSupervisoryControllerDSLPackage.Literals.ASSIGNMENT__VALUE);
	}

	@Check
	public void checkInitialValueType(Variable entity) {
		if (entity.getInitial() == null) return;
		
		ensureTypesEqual(entity.getType(), entity.getInitial(),
				RobotSupervisoryControllerDSLPackage.Literals.VARIABLE__INITIAL);
	}

	private void ensureTypeCorrect(EObject expression, TypesystemDataType<?> expected, EReference reference) {
		if (!typesystem.typeOf(expression).equals(expected)) {
			error("Incompatible types " + typesystem.typeOf(expression).getLabel() + " (expected " + expected.getLabel()
					+ ").", reference, INVALID_TYPE);
		}
	}

	private void ensureTypesEqual(EObject left, EObject right, EReference reference) {
		if (!typesystem.typeOf(left).equals(typesystem.typeOf(right))) {
			error("Incompatible types " + typesystem.typeOf(left).getLabel() + " and " + typesystem.typeOf(right).getLabel()
					+ ".", reference, INVALID_TYPE);
		}
	}

	private void ensureNumericTypeCorrect(EObject expression, EReference reference) {
		if (!typesystem.typeOf(expression).equals(TypesystemDataType.DOUBLE)
				&& !typesystem.typeOf(expression).equals(TypesystemDataType.INT)) {
			error("Incompatible types " + typesystem.typeOf(expression).getLabel() + " (expected "
					+ TypesystemDataType.INT.getLabel() + " or " + TypesystemDataType.DOUBLE.getLabel() + ").", reference,
					INVALID_TYPE);
		}
	}
}
