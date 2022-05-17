# Scenario 7 - Person following
In this scenario the robot follows a person using a machine learning technique (YOLOX). By using
the camera input feed, and calculating the offset from the center, it will follow the person
until it reaches an unsafe distance.

Next to the controller, the following nodes have been developed:

* Emergency stop, which emits `/stop` and `/continue` topics based on a user interface.

## ROS2
To start the simulation, first build the packages and source the install. Then, you can run the `simulate.launch.py` launch file:
```bash
cd yolox_ros && git apply ../yolox.patch && cd ..
pip3 -r yolox_ros/requirements.txt
vcs import < person_following.repos
colcon build --merge-install
source install/local_setup.bash
ros2 launch scenario simulate.launch.py
```

## Problems occurred during development
* There is no option to cast doubles or integers to each other.
* There is no option to count the length of an array.
* It is not possible to use conditional data within object values.

## References
Ge, Z., Liu, S., Wang, F., Li, Z., & Sun, J. (2021). YOLOX: Exceeding YOLO Series in 2021. arXiv preprint arXiv:2107. 08430.