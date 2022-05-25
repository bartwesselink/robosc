package nl.tue.robotsupervisorycontrollerdsl.generator.config.model;

public class SupervisorController {
	private SupervisorControllerLaunch launch = null;
	private SupervisorControllerRun run = null;

	public SupervisorControllerLaunch getLaunch() {
		return launch;
	}

	public void setLaunch(SupervisorControllerLaunch launch) {
		this.launch = launch;
	}

	public SupervisorControllerRun getRun() {
		return run;
	}

	public void setRun(SupervisorControllerRun run) {
		this.run = run;
	}

}
