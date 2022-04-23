package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.exception;

public class EarlyExitException extends RuntimeException {
	private static final long serialVersionUID = 1L;
	private int statusCode;

	public EarlyExitException(String errorMessage, Throwable err) {
        super(errorMessage, err);
    }

	public EarlyExitException(String errorMessage, int statusCode) {
        super(errorMessage);
        
        this.statusCode = statusCode;
    }
	
	public int getStatusCode() {
		return statusCode;
	}
}
