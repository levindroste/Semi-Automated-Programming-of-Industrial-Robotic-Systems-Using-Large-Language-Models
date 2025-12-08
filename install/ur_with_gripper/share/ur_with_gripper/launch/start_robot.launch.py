# NOTE: This launch file was originally for UR robot driver.
# ABB IRB 120 does not use the ur_robot_driver package.
# This file is kept as a placeholder for future real hardware integration.
# For simulation, use ur_sim_control.launch.py instead.

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_with_gripper",
            description="description package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur_with_gripper.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    log_message = LogInfo(
        msg="NOTE: IRB 120 real hardware driver is not yet implemented. "
        "Please use ur_sim_control.launch.py for simulation."
    )

    return LaunchDescription(declared_arguments + [log_message])
