#!/usr/bin/env python3
import subprocess
import time
import os


class ROS2Launcher:
    def __init__(self, workspace_path=None, domain_id=42, localhost_only=True):
        self.ws = os.path.expanduser(workspace_path or os.getcwd())
        self.tab_opened = False
        self.domain_id = domain_id
        self.localhost_only = localhost_only

    def _cmd(self, cmd, tab=None, wait=0):
        """FÃ¼hrt Befehl aus - entweder in Terminal-Tab oder direkt"""
        base = f"cd {self.ws} && source /opt/ros/humble/setup.bash && source install/setup.bash && {cmd}"

        if tab:
            term = f"gnome-terminal {'--tab' if self.tab_opened else ''} --title='{tab}' -- bash -c '{base}; read'"
            if not self.tab_opened:
                self.tab_opened = True
                wait = max(wait, 1.5)
            subprocess.Popen(term, shell=True)
        else:
            subprocess.run(base, shell=True, executable='/bin/bash')

        if wait > 0:
            time.sleep(wait)

    def run(self):
        print("ROS2 UR10e Setup")

        # Launch config
        cfg = "description_package:=ur_with_gripper description_file:=ur_with_gripper.urdf.xacro " \
              "ur_type:=ur10e runtime_config_package:=ur_with_gripper " \
              "controllers_file:=ur10e_controllers.yaml initial_joint_controller:=ur10e_arm_controller"

        # Start Gazebo with retry
        while True:
            self._cmd(f"ros2 launch ur_simulation_gz ur_sim_control.launch.py {cfg}", "Gazebo", 0)
            if input("\nRoboter in Gazebo/Rviz aufrecht? [j/n]: ").lower() == 'j':
                break
            print("Neustart...")
            time.sleep(2)

        # Launch sequence
        cmds = [
            ("ros2 launch ur_with_gripper_moveit_config spawn_controllers.launch.py", "Controllers", 2),
            ("RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG ros2 launch ur_with_gripper_moveit_config move_group.launch.py",
             "Move Group", 2),
            ("ros2 param set /move_group use_sim_time true", None, 2),
            ("ros2 launch ur_with_gripper_moveit_config moveit_rviz.launch.py", "MoveIt RViz", 3),
            ("ros2 param set /rviz use_sim_time true", None, 2),
            ("ros2 launch ur_with_gripper add_objects.launch.py", "Objects", 3)
        ]

        for cmd, tab, wait in cmds:
            self._cmd(cmd, tab, wait)

        print("\nSetup abgeschlossen!")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=None)
    parser.add_argument("--domain-id", type=int, default=42)  # ✅ NEU
    parser.add_argument("--localhost", action="store_true", default=True)  # ✅ NEU
    args = parser.parse_args()

    ROS2Launcher(
        workspace_path=args.workspace,
        domain_id=args.domain_id,
        localhost_only=args.localhost
    ).run()


#killall -9 gzserver gzclient gz ruby move_group rviz2 ros2
# Oder noch grÃ¼ndlicher:
#pkill -f ros2
#pkill -f gazebo
#pkill -f rviz