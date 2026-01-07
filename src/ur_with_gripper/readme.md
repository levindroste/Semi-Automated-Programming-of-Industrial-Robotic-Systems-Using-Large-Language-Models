# Documentation

ros2 run ur10e_control ur10e_control cartesian
ros2 run ur10e_hl_interface gripper_test
ros2 run ur10e_hl_interface cartesian_example
ros2 run ur10e_hl_interface sample_application

ros2 node kill /add_objects_node
ros2 launch ur_with_gripper add_objects.launch.py
ros2 run ur10e_hl_interface clipfix_bewegung

ros2 run tf2_ros tf2_echo panda_link0 panda_hand
ros2 run tf2_ros tf2_echo ur10e_base_link ur10e_tool0


Prompt ändern -> mit Beispielen -> Few-Shot learning

## Purpose

This package will contain everything related to simulating a gripper attached to a universal robot


## How to use

### Test the gripper alone

> ros2 launch custom_gripper gripper_control.launch.py
> ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "command: {position: -0.0, max_effort: 2.0}" -f 

position of 0.0 is opened
position of -0.045 is closed

### Launch the arm with the gripper and moveit (fake system)

> ros2 launch ur_with_gripper_moveit_config demo.launch.py

### Launch the arm with the gripper and moveit (gz simulation)

#### Launch the simulation

This will launch gazebo + controller_manager with the following controllers pre-loaded:
- ur10e_arm_controller
- joint_state_broadcaster

ros2 launch ur_simulation_gz ur_sim_control.launch.py  description_package:=ur_with_gripper description_file:=ur_with_gripper.urdf.xacro ur_type:=ur10e runtime_config_package:=ur_with_gripper controllers_file:=ur10e_controllers.yaml initial_joint_controller:=ur10e_arm_controller

#### Activate the controllers

The gripper controller is still not loaded nor active.
This next command will load and activate it.

ros2 launch ur_with_gripper_moveit_config spawn_controllers.launch.py


#### Launch MoveGroup and make it use sim time

ros2 launch ur_with_gripper_moveit_config move_group.launch.py
ros2 param set /move_group use_sim_time true

Launch moveit rviz and make rviz use sim time

ros2 launch ur_with_gripper_moveit_config moveit_rviz.launch.py
ros2 param set /rviz use_sim_time true

ros2 launch ur_with_gripper add_objects.launch.py
ros2 run ur10e_hl_interface clipfix_bewegung

## Pick and place

### We can use pymoveit2 in python

and example: 

> ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57]"
