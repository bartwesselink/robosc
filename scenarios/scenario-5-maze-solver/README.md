# Scenario 5 - Maze solver
In this scenario, the robot uses the right-wall follower algorithm to find the exit of a maze. The layout of the maze
equals the layout that was used by [Andrew Yong](https://andrewyong7338.medium.com/maze-escape-with-wall-following-algorithm-170c35b88e00). The algorithm itself follows from a description found [here](https://en.wikipedia.org/wiki/Maze-solving_algorithm),

Next to the controller, the following nodes have been developed:

* Emergency stop, which emits `/stop` and `/continue` topics based on a user interface.
* Fixed rotator, which can rotate a relative amount of degrees, depending on the data that is sent to the topic

## ROS2
To start the simulation, first build the packages and source the install. Then, you can run the `simulate.launch.py` launch file:
```bash
colcon build --merge-install
source install/local_setup.bash
ros2 launch scenario simulate.launch.py
```

## Problems occurred during development
* Due to a CIF-limitation it is not possible to have overlapping enum values
* There is no check for uniqueness on variable names
* It could come in handy to be able to define global constants (e.g. for thresholds)
