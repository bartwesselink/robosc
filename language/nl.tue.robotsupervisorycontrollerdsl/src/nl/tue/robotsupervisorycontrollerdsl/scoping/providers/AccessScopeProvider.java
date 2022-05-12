package nl.tue.robotsupervisorycontrollerdsl.scoping.providers;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.scoping.IScope;
import org.eclipse.xtext.scoping.Scopes;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Access;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AccessType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.AccessibleItem;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ArrayDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Component;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumValue;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.FeedbackResultType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RequestResultType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResponseResultType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ResultTransition;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.State;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Variable;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.impl.AccessTypeImpl;
import nl.tue.robotsupervisorycontrollerdsl.scoping.common.AbstractScopeProvider;

public class AccessScopeProvider extends AbstractScopeProvider {
	@Override
	public IScope getScope(EObject context, EReference reference) {
		if (context instanceof Access && reference == RobotSupervisoryControllerDSLPackage.Literals.ACCESS__FIRST_ITEM) {
			List<EObject> candidates = new ArrayList<>();

			Component parentComponent = ModelHelper.findParentOfType(context, Component.class);
			Robot robot = ModelHelper.findParentOfType(context, Robot.class);

			candidates.addAll(ModelHelper.findChildren(robot, Component.class).stream().map(it -> (EObject) it)
					.collect(Collectors.toList()));

			if (parentComponent != null) {
				candidates.addAll(ModelHelper.findChildren(parentComponent, Variable.class).stream()
						.map(it -> (EObject) it).collect(Collectors.toList()));
			}

			candidates.addAll(ModelHelper.findChildren(robot, EnumValue.class).stream().map(it -> (EObject) it)
					.collect(Collectors.toList()));

			return Scopes.scopeFor(candidates);
		} else if (context instanceof AccessType && reference == RobotSupervisoryControllerDSLPackage.Literals.ACCESS_TYPE__ITEM) {
			Access access = ModelHelper.findParentOfType(context, Access.class);
			List<EObject> candidates = new ArrayList<>();

			if (access.getFirstItem() != null) {
				AccessibleItem first = access.getFirstItem();

				if (first instanceof Component) {
					candidates.addAll(ModelHelper.findInComponent((Component) first, Variable.class).stream().map(it -> (EObject) it)
							.collect(Collectors.toList()));

					candidates.addAll(ModelHelper.findInComponent((Component) first, State.class).stream().map(it -> (EObject) it)
							.collect(Collectors.toList()));
				}
			} else if (access.getValue() != null) {
				EnumDataType parentEnum = ModelHelper.findParentOfType(context, EnumDataType.class);
				ResultTransition parentResultTransition = ModelHelper.findParentOfType(context, ResultTransition.class);

				DataType dataType = null;

				if (parentEnum != null) {
					dataType = parentEnum.getType();
				} else if (parentResultTransition != null) {
					CommunicationType communicationType = parentResultTransition.getCommunicationType();

					if (communicationType instanceof Service && parentResultTransition.getResultType() instanceof RequestResultType) {
						dataType = ((Service) communicationType).getRequestType();
					} else if (communicationType instanceof Service && parentResultTransition.getResultType() instanceof ResponseResultType) {
						dataType = ((Service) communicationType).getResponseType();
					}

					if (communicationType instanceof Action && parentResultTransition.getResultType() instanceof RequestResultType) {
						dataType = ((Action) communicationType).getRequestType();
					} else if (communicationType instanceof Action && parentResultTransition.getResultType() instanceof ResponseResultType) {
						dataType = ((Action) communicationType).getResponseType();
					} else if (communicationType instanceof Action && parentResultTransition.getResultType() instanceof FeedbackResultType) {
						dataType = ((Action) communicationType).getFeedbackType();
					}

					if (communicationType instanceof Message) {
						dataType = ((Message) communicationType).getType();
					}
				}
				
				for (AccessType type : access.getTypes()) {
					AccessTypeImpl implementation = (AccessTypeImpl) type;
	
					if (implementation.basicGetItem() != null) {
						if (implementation.basicGetItem() instanceof ObjectProperty) {
							dataType = ((ObjectProperty) implementation.basicGetItem()).getType();
						}
					} else if (implementation.basicGetItem() == null && dataType instanceof ArrayDataType) {
						dataType = ((ArrayDataType) dataType).getType();
					}
				}

				if (dataType instanceof ComplexDataTypeReference) {
					ComplexDataTypeReference typeReference = (ComplexDataTypeReference) dataType;

					candidates.addAll(ModelHelper.findChildren(typeReference.getType(), ObjectProperty.class).stream()
							.map(it -> (EObject) it).collect(Collectors.toList()));
				}
			}
		
			return Scopes.scopeFor(candidates);
		}

		return Scopes.scopeFor(new ArrayList<>());
	}

	@Override
	public boolean supports(EObject entity, EReference reference) {
		return (entity instanceof Access
				&& reference == RobotSupervisoryControllerDSLPackage.Literals.ACCESS__FIRST_ITEM)
				|| (entity instanceof AccessType && reference == RobotSupervisoryControllerDSLPackage.Literals.ACCESS_TYPE__ITEM);
	}
}