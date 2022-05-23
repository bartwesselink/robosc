package nl.tue.robotsupervisorycontrollerdsl.generator.config.model;

public class Output {
	private String ros1NodeLocation = null;
	private String ros2NodeLocation = null;

	public String getRos1NodeLocation() {
		return ros1NodeLocation;
	}

	public void setRos1NodeLocation(String ros1NodeLocation) {
		this.ros1NodeLocation = ros1NodeLocation;
	}

	public String getRos2NodeLocation() {
		return ros2NodeLocation;
	}

	public void setRos2NodeLocation(String ros2NodeLocation) {
		this.ros2NodeLocation = ros2NodeLocation;
	}
}
