# Scenario 2 - Simple navigatation
This scenario allows navigation of the robot using the Nav2-package. RVIZ will be started, and once an initial pose has been set, you can publish a point that the controller will navigate to. To run this example, it is required to have the Nav2-package installed.

Next to the controller, the following nodes have been developed:

* Emergency stop, which emits `/stop` and `/continue` topics based on a user interface.

## ROS2
To start the simulation, first build the packages and source the install. Then, you can run the `simulate.launch.py` launch file:
```bash
colcon build --merge-install
source install/local_setup.bash
ros2 launch scenario simulate.launch.py
```

## Problems occurred during development
* There was no option to cancel an action, that the emergency stop uses.
* It was impossible to determine an action error
* Support for multiple assignments should be added
* Variables do not support complex data types
