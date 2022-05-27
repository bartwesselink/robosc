# Scenario 8 - Supervisor
In this scenario the robot follows a line, but in this case it will use a supervisor. First, there is a controller that will perform the line following. The supervisor adds functionality for the emergency stop.

Next to the controller, the following nodes have been developed:

* Emergency stop, which emits `/stop` and `/continue` topics based on a user interface.
* Simple movement, that allows the `/move_forward` command, which will move the robot at a given velocity for 100ms.

## ROS2
To start the simulation, first build the packages and source the install. Then, you can run the `simulate.launch.py` launch file:
```bash
vcs import < person_following.repos
colcon build --merge-install
source install/local_setup.bash
ros2 launch scenario simulate.launch.py
```
