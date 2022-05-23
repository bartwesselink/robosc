package nl.tue.robotsupervisorycontrollerdsl.generator.config.model;

public class Config {
	private Output output = new Output();
	private boolean publishStateInformation = true;

	public Output getOutput() {
		return output;
	}

	public void setOutput(Output output) {
		this.output = output;
	}

	public boolean isPublishStateInformation() {
		return publishStateInformation;
	}

	public void setPublishStateInformation(boolean publishStateInformation) {
		this.publishStateInformation = publishStateInformation;
	}
}
