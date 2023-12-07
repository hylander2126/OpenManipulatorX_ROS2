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
Or for python:
```
ros2 run rbe500_example_py basic_robot_control
```
## Forward Kinematics
Prints the end-effector position, reading the current joint values. 
To run the forward kinematics node:
```
ros2 run rbe500_example_py forward_kineatics
```
## Inverse Kinematics
Moves the end-effector to the desired position.
To run the inverse kinematics node:
```
ros2 run rbe500_example_py inverse_kineatics
```
Video link for pick and place of an object, performed using inverse kinematics - https://youtube.com/shorts/_iANV6D9pfc

## Velocity Kinematics
Calculates the end-effector velocity by reading the joint velocities and vice versa using a Jacobian matrix.
```
ros2 run rbe500_example_py velocity_kineatics
```
To validate the velocity kinematics, a discrete velocity controller is implemented. Constant end-effector velocity is given as input, to run the node:
```
ros2 run rbe500_example_py discrete_velocity_control
```
Video link to show the end-effector moving in a straight line - https://youtu.be/-0vHnbNme9Y 

To visualize the end-effector position run:
```
cd ~/rbe500_example_py
python3 visualization.py
```
---
RBE 500 - Foundations of Robotics 2023 taught by Professor Berk Calli at Worcester Polytechnic Institute Robotics Engineering Department
