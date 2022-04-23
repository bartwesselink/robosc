package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.exception;

public class CIFException extends RuntimeException {
	private static final long serialVersionUID = 2L;

	public CIFException(String errorMessage) {
        super(errorMessage);
    }
}
