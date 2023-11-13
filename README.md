# OpenManipulatorX_ROS2
ROS2 Humble packages to control Open Manipulator X

## Build
Clone the entire repo into the src/ directory in a new ROS2 workspace.

Run the following to build the packages in your new workspace:
```
colcon build --symlink-install
```
You may get some warning mesages, ignore them for now as long as everything successfully builds.


Don't forget to source your workspace:
```
source install/setup.bash
```

After connecting to the robot over USB, run the following to begin position control:
```
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```

Now run the example code to move the robot:
```
ros2 run rbe500-example basic_robot_control
```


---
RBE 500 - Foundations of Robotics 2023 taught by Professor Berk Calli at Worcester Polytechnic Institute Robotics Engineering Department
