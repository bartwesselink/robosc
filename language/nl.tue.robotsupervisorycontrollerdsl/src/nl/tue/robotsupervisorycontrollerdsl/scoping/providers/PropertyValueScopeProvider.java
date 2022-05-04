package nl.tue.robotsupervisorycontrollerdsl.scoping.providers;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.scoping.IScope;
import org.eclipse.xtext.scoping.Scopes;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.ModelHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Action;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.CommunicationType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Message;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectPropertyValue;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ProvideStatement;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.RobotSupervisoryControllerDSLPackage;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Service;
import nl.tue.robotsupervisorycontrollerdsl.scoping.common.AbstractScopeProvider;

public class PropertyValueScopeProvider extends AbstractScopeProvider {	
	@Override
	public IScope getScope(EObject context, EReference reference) {
		ProvideStatement provideStatement = ModelHelper.findParentOfType(context, ProvideStatement.class);
		ObjectPropertyValue parentPropertyValue = ModelHelper.findParentOfType(context.eContainer(),
				ObjectPropertyValue.class);

		DataType dataType = null;

		if (parentPropertyValue != null) {
			dataType = parentPropertyValue.getProperty().getType();
		} else if (provideStatement != null) {
			CommunicationType communicationType = provideStatement.getCommunicationType();

			if (communicationType instanceof Service) {
				dataType = ((Service) communicationType).getRequestType();
			}

			if (communicationType instanceof Action) {
				dataType = ((Action) communicationType).getRequestType();
			}

			if (communicationType instanceof Message) {
				dataType = ((Message) communicationType).getType();
			}
		}

		if (dataType instanceof ComplexDataTypeReference) {
			ComplexDataTypeReference typeReference = (ComplexDataTypeReference) dataType;
			List<EObject> properties = ModelHelper.findChildren(typeReference.getType(), ObjectProperty.class)
					.stream().map(it -> (EObject) it).collect(Collectors.toList());

			return Scopes.scopeFor(properties);
		}

		return Scopes.scopeFor(new ArrayList<>());
	}

	@Override
	public boolean supports(EObject entity, EReference reference) {
		return entity instanceof ObjectPropertyValue
				&& reference == RobotSupervisoryControllerDSLPackage.Literals.OBJECT_PROPERTY_VALUE__PROPERTY;
	}

}