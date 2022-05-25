package nl.tue.robotsupervisorycontrollerdsl.generator.config.model;

public class Supervisor {
	private SupervisorController controller = new SupervisorController();

	public SupervisorController getController() {
		return controller;
	}

	public void setController(SupervisorController controller) {
		this.controller = controller;
	}

}
