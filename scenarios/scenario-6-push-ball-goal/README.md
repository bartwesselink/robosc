# Scenario 6 - Push ball in goal
In this scenario the robot attempts to find a ball (it will rotate until it has found a ball). Then it will move towards
a green goal. If it finds that the ball is not within reasonable bounds in front of the robot, it will correct it. It is 
inspired by the video of [RobotClub Malaysia](
https://www.youtube.com/watch?v=nCL6YlAlYm4&t=2s).

Next to the controller, the following nodes have been developed:

* Emergency stop, which emits `/stop` and `/continue` topics based on a user interface.
* Ball detector, which publishes to `/ball_correction`, `/no_ball`, `/needs_adjustment`, `/no_adjustment` and `/ball_front_check` topics
* Goal detector, which publishes to `/goal_correction` and `/no_goal` topics

## ROS2
To start the simulation, first build the packages and source the install. Then, you can run the `simulate.launch.py` launch file:
```bash
colcon build --merge-install
source install/local_setup.bash
ros2 launch scenario simulate.launch.py
```

## Problems occurred during development
* State names are not scoped to their current automaton when transitioning from one state to the other.
* Initial values of variables can conflict with the names of state within an automaton.
