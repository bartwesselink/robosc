# Scenario 1 - Line follower
This scenario contains a sample controller where a robot follows a line. It is based on the tutorial from [Gaitech](http://edu.gaitech.hk/turtlebot/line-follower.html), and uses the Gazebo
world as provided in https://github.com/sudrag/line_follower_turtlebot.

Next to the controller, the following nodes have been developed:

* Emergency stop, which emits `/stop` and `/continue` topics based on a user interface.
* Line detector, which emits `/correction` and `/no_line` based on the camera feed coming from the robot

## ROS2
To start the simulation, first build the packages and source the install. Then, you can run the `simulate.launch.py` launch file:
```bash
colcon build --merge-install
source install/local_setup.bash
ros2 launch scenario simulate.launch.py
```

## Problems occurred during development
* The language does not contain an option to have a negative variable.
* The language specifies duplicate dependencies.
* Topic validation can be added to check whether topics follow a valid format.
* Data provisioning was not working properly with complex object.
* Library references yielded errors.
* Allow libraries to provide data