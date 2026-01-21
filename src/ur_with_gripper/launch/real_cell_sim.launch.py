"""
Launch file for simulating the real robot cell in Gazebo.

This launches:
1. Gazebo simulation with IRB 120 robot
2. MoveIt2 for motion planning
3. RViz for visualization
4. Collision objects from real_robot_cell.aml

Usage:
    ros2 launch ur_with_gripper real_cell_sim.launch.py

After verifying trajectory in simulation, use execute_on_real_robot.py
to run the same motion on the real robot.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    aml_file = LaunchConfiguration("aml_file")
    cell_mode = LaunchConfiguration("cell_mode")

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Save URDF for debugging
    with open("/tmp/irb120_real_cell_debug.urdf", 'w') as f:
        f.write(robot_description_content.perform(context))

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # MoveIt Configuration (needed for RViz and move_group)
    moveit_config = MoveItConfigsBuilder(
        "ur", package_name="ur_with_gripper_moveit_config"
    ).to_moveit_configs()

    # RViz needs MoveIt parameters to display the planning scene
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after joint_state_broadcaster
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # Joint controller
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # Gazebo nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "irb120_with_gripper",
            "-allow_renaming",
            "true",
        ],
    )

    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )

    gz_launch_description_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # Clock bridge for ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    # Move Group Node for motion planning
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # Delay move_group to ensure robot_state_publisher and controllers are ready
    delayed_move_group = TimerAction(
        period=3.0,
        actions=[move_group_node],
    )

    # Get paths for add_objects_node
    ur10e_hl_interface_dir = get_package_share_directory('ur10e_hl_interface')
    mesh_dir = os.path.join(ur10e_hl_interface_dir, 'meshes')

    # Resolve AML file path
    aml_file_resolved = aml_file.perform(context)
    if not os.path.isabs(aml_file_resolved):
        aml_file_resolved = os.path.join(ur10e_hl_interface_dir, 'config', aml_file_resolved)

    # Resolve cell_mode
    cell_mode_resolved = cell_mode.perform(context)

    # Add collision objects from AML file (delayed to ensure MoveIt is ready)
    # Using Python script for better debugging and reliability
    add_objects_node = Node(
        package='ur10e_hl_interface',
        executable='add_collision_objects.py',
        name='add_collision_objects',
        output='screen',
        parameters=[{
            'mesh_directory': mesh_dir,
            'aml_file': aml_file_resolved,
            'cell_mode': cell_mode_resolved,
            'use_sim_time': True,
        }]
    )

    # Delay add_objects_node to ensure MoveIt planning scene is ready
    # (needs more time since move_group starts at 3s)
    delayed_add_objects = TimerAction(
        period=8.0,
        actions=[add_objects_node],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_launch_description_without_gui,
        gz_sim_bridge,
        delayed_move_group,
        delayed_add_objects,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_with_gripper",
            description='Package with the controller\'s configuration in "config" folder.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="irb120_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_with_gripper",
            description="Description package with robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur_with_gripper.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="irb120_arm_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start gazebo with GUI?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aml_file",
            default_value="wuerfel_config.aml",
            description="AML file defining collision objects for the robot cell.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cell_mode",
            default_value="wuerfel",
            description="Cell mode: 'wuerfel' for cube grid, 'klemmen' for terminal blocks.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
