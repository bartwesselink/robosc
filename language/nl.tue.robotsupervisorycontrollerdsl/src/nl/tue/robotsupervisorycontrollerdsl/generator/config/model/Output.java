package nl.tue.robotsupervisorycontrollerdsl.generator.config.model;

public class Output {
	private String ros1ControllerNodeLocation = null;
	private String ros2ControllerNodeLocation = null;
	private String ros2SupervisorNodeLocation = null;

	public String getRos1ControllerNodeLocation() {
		return ros1ControllerNodeLocation;
	}

	public void setRos1ControllerNodeLocation(String ros1ControllerNodeLocation) {
		this.ros1ControllerNodeLocation = ros1ControllerNodeLocation;
	}

	public String getRos2ControllerNodeLocation() {
		return ros2ControllerNodeLocation;
	}

	public void setRos2ControllerNodeLocation(String ros2ControllerNodeLocation) {
		this.ros2ControllerNodeLocation = ros2ControllerNodeLocation;
	}

	public String getRos2SupervisorNodeLocation() {
		return ros2SupervisorNodeLocation;
	}

	public void setRos2SupervisorNodeLocation(String ros2SupervisorNodeLocation) {
		this.ros2SupervisorNodeLocation = ros2SupervisorNodeLocation;
	}

}
