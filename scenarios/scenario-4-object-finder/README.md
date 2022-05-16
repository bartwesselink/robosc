# Scenario 4 - Object finder
In this scenario, the robot follows a wall and attempts to find an object, namely a stop sign.
This detection happens using `darknet_ros`, a package that can perform object classification based on a camera input stream.

Next to the controller, the following nodes have been developed:

* Emergency stop, which emits `/stop` and `/continue` topics based on a user interface.
* Fixed rotator, which can rotate a relative 90 degrees

## ROS2
To start the simulation, first build the packages and source the install. Then, you can run the `simulate.launch.py` launch file:
```bash

vcs import . < object_finder.repos
colcon build --merge-install
source install/local_setup.bash
ros2 launch scenario simulate.launch.py
```

## Problems occurred during development
* It is not possible to re-use transformation functions of enums
* Imports from packages (for messages, actions and services) must sometimes be defined multiple times
* Visualization of multiple states during debugging did not work and yielded errors
* The use of strings should be limited to code-only variables 

## References
M. Bjelonic "YOLO ROS: Real-Time Object Detection for ROS", URL: https://github.com/leggedrobotics/darknet_ros, 2018.

