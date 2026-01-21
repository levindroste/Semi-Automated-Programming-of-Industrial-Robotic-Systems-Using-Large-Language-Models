"""
Launch file for executing the LLM-generated script.

This launch file provides the MoveIt configuration parameters to the script node
so it can connect to the running move_group.

Usage:
    First start the simulation:
        ros2 launch ur_with_gripper real_cell_sim.launch.py

    Then run this script (simulation only):
        ros2 launch ur10e_hl_interface execute_script.launch.py

    Or run with real robot:
        ros2 launch ur10e_hl_interface execute_script.launch.py real_robot:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    real_robot_arg = DeclareLaunchArgument(
        'real_robot',
        default_value='false',
        description='Enable real robot mode (sends commands via socket)'
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.125.1',
        description='Real robot IP address'
    )
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='5000',
        description='Real robot socket port'
    )
    cell_mode_arg = DeclareLaunchArgument(
        'cell_mode',
        default_value='wuerfel',
        description='Cell mode: wuerfel or klemmen'
    )

    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "ur", package_name="ur_with_gripper_moveit_config"
    ).to_moveit_configs()

    # Get path to AML config file based on cell_mode
    ur10e_hl_interface_dir = get_package_share_directory('ur10e_hl_interface')
    # Note: LaunchConfiguration can't be used directly in os.path.join at launch time
    # We pass cell_mode to the node and let it construct the path
    aml_file_wuerfel = os.path.join(ur10e_hl_interface_dir, 'config', 'wuerfel_config.aml')
    aml_file_klemmen = os.path.join(ur10e_hl_interface_dir, 'config', 'klemmen_config.aml')

    # Script execution node
    script_node = Node(
        package='ur10e_hl_interface',
        executable='script',
        name='robot_hl_interface_simple',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
            {"aml_file_wuerfel": aml_file_wuerfel},
            {"aml_file_klemmen": aml_file_klemmen},
            {"cell_mode": LaunchConfiguration('cell_mode')},
            {"real_robot": LaunchConfiguration('real_robot')},
            {"robot_ip": LaunchConfiguration('robot_ip')},
            {"robot_port": LaunchConfiguration('robot_port')},
        ],
    )

    return LaunchDescription([
        real_robot_arg,
        robot_ip_arg,
        robot_port_arg,
        cell_mode_arg,
        script_node
    ])
