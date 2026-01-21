#!/usr/bin/env python3
import subprocess
import time
import os


class ROS2Launcher:
    def __init__(self, workspace_path=None, cell_mode='wuerfel'):
        self.ws = os.path.expanduser(workspace_path or os.getcwd())
        self.tab_opened = False
        self.cell_mode = cell_mode
        self.aml_file = 'klemmen_config.aml' if cell_mode == 'klemmen' else 'wuerfel_config.aml'

    def kill_existing_processes(self):
        """Kill all existing ROS/Gazebo processes before starting new ones"""
        print("Killing existing processes...")
        killall_cmd = "killall -9 ruby rviz2 gzserver gzclient gz move_group 2>/dev/null"
        subprocess.run(killall_cmd, shell=True, executable='/bin/bash')
        pkill_cmd = "pkill -9 -f 'gz sim'; pkill -9 -f 'rviz'; pkill -9 -f 'move_group'; pkill -9 -f 'parameter_bridge'; pkill -9 -f 'robot_state_publisher'; pkill -9 -f 'spawner'; pkill -9 -f 'add_collision'"
        subprocess.run(pkill_cmd, shell=True, executable='/bin/bash', stderr=subprocess.DEVNULL)
        time.sleep(3)
        print("Processes killed.")

    def build_workspace(self):
        """Build the ROS2 workspace"""
        print("Building workspace...")
        build_cmd = f"cd {self.ws} && source /opt/ros/humble/setup.bash && colcon build --symlink-install"
        result = subprocess.run(build_cmd, shell=True, executable='/bin/bash', capture_output=True, text=True)
        if result.returncode == 0:
            print("Build successful!")
        else:
            print(f"Build failed: {result.stderr}")
            return False
        return True

    def _cmd(self, cmd, tab=None, wait=0):
        """Executes command - either in terminal tab or directly"""
        base = f"cd {self.ws} && source /opt/ros/humble/setup.bash && source install/setup.bash && {cmd}"

        if tab:
            # Keep terminal open with 'exec bash' instead of 'read'
            term = f"gnome-terminal {'--tab' if self.tab_opened else ''} --title='{tab}' -- bash -c '{base}; exec bash'"
            if not self.tab_opened:
                self.tab_opened = True
                wait = max(wait, 1.5)
            subprocess.Popen(term, shell=True)
        else:
            subprocess.run(base, shell=True, executable='/bin/bash')

        if wait > 0:
            time.sleep(wait)

    def run(self):
        print("ROS2 ABB IRB 120 Setup")

        # Kill existing processes first
        self.kill_existing_processes()

        # Build the workspace
        if not self.build_workspace():
            print("Build failed, aborting.")
            return

        # IRB 120 specific launch configuration
        cfg = "description_package:=ur_with_gripper description_file:=ur_with_gripper.urdf.xacro " \
              "runtime_config_package:=ur_with_gripper " \
              "controllers_file:=irb120_controllers.yaml initial_joint_controller:=irb120_arm_controller " \
              f"cell_mode:={self.cell_mode} aml_file:={self.aml_file}"

        # Start simulation with retry loop
        while True:
            self._cmd(f"ros2 launch ur_with_gripper real_cell_sim.launch.py {cfg}", "Simulation", 0)
            response = input("\nRobot visible in Gazebo/RViz and standing upright? [y/n]: ").lower()
            if response in ['y', 'j', 'yes', 'ja']:
                break
            print("Restarting...")
            self.kill_existing_processes()
            time.sleep(2)

        print("\n" + "="*50)
        print("Setup complete!")
        print("="*50)
        print("\nYou can now use the GUI to execute robot programs.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=None)
    parser.add_argument("--cell-mode", default="wuerfel", choices=["wuerfel", "klemmen"])
    args = parser.parse_args()

    ROS2Launcher(workspace_path=args.workspace, cell_mode=args.cell_mode).run()
