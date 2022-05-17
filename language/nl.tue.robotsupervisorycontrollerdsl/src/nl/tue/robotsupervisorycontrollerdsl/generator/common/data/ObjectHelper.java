package nl.tue.robotsupervisorycontrollerdsl.generator.common.data;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ArrayDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BasicDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ComplexDataTypeReference;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.EnumDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Expression;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectProperty;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectPropertyValue;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectPropertyValueContent;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.ObjectValue;

@Singleton
public class ObjectHelper {
	public List<PropertyIdentifierPair> flattenProperties(ObjectDataType type, String prefix) {
		List<PropertyIdentifierPair> result = new ArrayList<>();
		
		for (ObjectProperty property : type.getProperties()) {
			if (property.getType() instanceof BasicDataType) {
				result.add(new PropertyIdentifierPair(property, prefix + property.getName()));
			} else if (property.getType() instanceof ComplexDataTypeReference) {
				ComplexDataTypeReference reference = (ComplexDataTypeReference) property.getType();
				ComplexDataType referenced = reference.getType();
				
				if (referenced instanceof ObjectDataType) {				
					result.addAll(
							flattenProperties((ObjectDataType) referenced, prefix + property.getName() + '_')
						);
				} else if (referenced instanceof EnumDataType) {
					result.add(new PropertyIdentifierPair(property, prefix + property.getName()));
				}
			} else if (property.getType() instanceof ArrayDataType) {
				ArrayDataType array = (ArrayDataType) property.getType();
				DataType arrayType = array.getType();
				
				if (arrayType instanceof ComplexDataTypeReference) {	
					ComplexDataTypeReference reference = (ComplexDataTypeReference) arrayType;
					ComplexDataType referenced = reference.getType();
					
					if (referenced instanceof ObjectDataType) {				
						result.addAll(
								flattenProperties((ObjectDataType) referenced, "")
							);
					}
				}
			}
		}
		
		return result;
	}

	public List<PropertyIdentifierPair> flattenProperties(ObjectValue type, String prefix) {
		List<PropertyIdentifierPair> result = new ArrayList<>();
		
		for (ObjectPropertyValue propertyValue : type.getProperties()) {
			ObjectProperty property = propertyValue.getProperty();
			ObjectPropertyValueContent content = propertyValue.getValue();
			
			if (content instanceof Expression) {
				result.add(new PropertyIdentifierPair(property, prefix + property.getName()));
			} else if (content instanceof ObjectValue) {
				result.addAll(
					flattenProperties((ObjectValue) content, prefix + property.getName() + '_')
				);
			}
		}
		
		return result;
	}
}
