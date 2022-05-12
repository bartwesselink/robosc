# Scenario 3 - Obstacle navigatation
This scenario allows navigation of the robot using the Nav2-package. In this case, opposite to scenario 2,
there will be obstacles that the robot needs to navigate around.
RVIZ will be started, and once an initial pose has been set, you can publish a point that the controller will navigate to. To run this example, it is required to have the Nav2-package installed.

Next to the controller, the following nodes have been developed:

* Emergency stop, which emits `/stop` and `/continue` topics based on a user interface.

## ROS2
To start the simulation, first build the packages and source the install. Then, you can run the `simulate.launch.py` launch file:
```bash
colcon build --merge-install
source install/local_setup.bash
ros2 launch scenario simulate.launch.py
```

