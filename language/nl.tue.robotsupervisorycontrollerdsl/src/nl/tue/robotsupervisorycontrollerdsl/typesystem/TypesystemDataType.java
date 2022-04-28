package nl.tue.robotsupervisorycontrollerdsl.typesystem;

import org.eclipse.emf.ecore.EObject;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.BooleanDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.DoubleDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.IntegerDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.NoneDataType;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.StringDataType;

public class TypesystemDataType<T extends EObject> {
	public static TypesystemDataType<IntegerDataType> INT = new TypesystemDataType<>(IntegerDataType.class, null, "int");
	public static TypesystemDataType<StringDataType> STRING = new TypesystemDataType<>(StringDataType.class, null, "string");
	public static TypesystemDataType<BooleanDataType> BOOLEAN = new TypesystemDataType<>(BooleanDataType.class, null, "boolean");
	public static TypesystemDataType<DoubleDataType> DOUBLE = new TypesystemDataType<>(DoubleDataType.class, null, "double");
	public static TypesystemDataType<NoneDataType> NONE = new TypesystemDataType<>(NoneDataType.class, null, "none");
	
	private Class<T> primitive;
	private EObject referenced = null;
	private String label = null;

	public TypesystemDataType(Class<T> primitive, EObject referenced, String label) {
		this.primitive = primitive;
		this.referenced = referenced;
		this.label = label;
	}
	
	public Class<T> getPrimitive() {
		return primitive;
	}
	
	public EObject getReferenced() {
		return referenced;
	}
	
	public String getLabel() {
		return label;
	}
	
	@Override
	public boolean equals(Object obj) {
		if (!(obj instanceof TypesystemDataType)) {
			return false;
		}
		
		TypesystemDataType<?> other = (TypesystemDataType<?>) obj;
		
		return this.primitive == other.getPrimitive()
				&& this.referenced == other.getReferenced();
	}
}
