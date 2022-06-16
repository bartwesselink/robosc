package nl.tue.robotsupervisorycontrollerdsl.generator.config.model;

public class Config {
	private Output output = new Output();
	private boolean publishStateInformation = true;
	private Supervisor supervisor = null;
	private boolean writeEventsToLog = false;

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

	public Supervisor getSupervisor() {
		return supervisor;
	}

	public void setSupervisor(Supervisor supervisor) {
		this.supervisor = supervisor;
	}

	public boolean isWriteEventsToLog() {
		return writeEventsToLog;
	}

	public void setWriteEventsToLog(boolean writeEventsToLog) {
		this.writeEventsToLog = writeEventsToLog;
	}
}
