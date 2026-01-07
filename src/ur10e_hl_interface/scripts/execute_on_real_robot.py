#!/usr/bin/env python3
"""
Execute trajectories on the real ABB IRB 120 robot via socket communication.

This script connects to the IRC5 controller and executes joint trajectories
that were planned in MoveIt2 simulation.

Usage:
    # Interactive mode - prompts for target position
    ros2 run ur10e_hl_interface execute_on_real_robot.py

    # Execute specific joint positions (degrees)
    ros2 run ur10e_hl_interface execute_on_real_robot.py --joints 0 0 0 0 0 0

    # Move to home position
    ros2 run ur10e_hl_interface execute_on_real_robot.py --home

Prerequisites:
    - SocketServer.mod must be running on the IRC5 controller
    - Robot must be in AUTO mode with Motors On
"""

import socket
import time
import argparse
import math
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Import RAPID converter
from trajectory_to_rapid import TrajectoryToRAPID


class RealRobotExecutor(Node):
    """Execute trajectories on real ABB IRB 120 via socket."""

    # Joint names matching the URDF
    JOINT_NAMES = [
        'irb120_joint_1',
        'irb120_joint_2',
        'irb120_joint_3',
        'irb120_joint_4',
        'irb120_joint_5',
        'irb120_joint_6',
    ]

    # Home position in degrees
    HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def __init__(self, robot_ip='192.168.125.1', robot_port=5000):
        super().__init__('real_robot_executor')

        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.socket = None
        self.connected = False

        # Publisher for joint states (to update RViz)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/real_robot/joint_states',
            10
        )

        self.get_logger().info(f'Real Robot Executor initialized')
        self.get_logger().info(f'Robot address: {robot_ip}:{robot_port}')

    def connect(self) -> bool:
        """Connect to the robot socket server."""
        if self.connected:
            return True

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)
            self.socket.connect((self.robot_ip, self.robot_port))
            self.connected = True
            self.get_logger().info(f'Connected to robot at {self.robot_ip}:{self.robot_port}')

            # Test connection
            response = self._send_command('PING')
            if 'PONG' in response:
                self.get_logger().info('Connection verified (PONG received)')
                return True
            else:
                self.get_logger().warn(f'Unexpected response to PING: {response}')
                return True  # Still connected, just unexpected response

        except socket.timeout:
            self.get_logger().error('Connection timeout - is SocketMain running on the robot?')
            return False
        except ConnectionRefusedError:
            self.get_logger().error('Connection refused - check robot IP and port')
            return False
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            return False

    def disconnect(self):
        """Disconnect from the robot."""
        if self.socket:
            try:
                self._send_command('QUIT')
            except:
                pass
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
            self.connected = False
            self.get_logger().info('Disconnected from robot')

    def _send_command(self, cmd: str, timeout: float = 30.0) -> str:
        """Send command to robot and receive response."""
        if not self.connected or not self.socket:
            raise RuntimeError('Not connected to robot')

        self.socket.settimeout(timeout)
        self.socket.send((cmd + '\n').encode())
        time.sleep(0.1)  # Small delay for robot to process

        response = self.socket.recv(1024).decode().strip()
        return response

    def get_current_position(self) -> list:
        """Get current joint positions in degrees."""
        response = self._send_command('GETPOS')

        if response.startswith('POS:'):
            parts = response[4:].split()
            joints = [float(x) for x in parts]
            self.get_logger().info(f'Current position (deg): {joints}')
            return joints
        else:
            self.get_logger().error(f'Failed to get position: {response}')
            return None

    def get_current_position_radians(self) -> list:
        """Get current joint positions in radians."""
        joints_deg = self.get_current_position()
        if joints_deg:
            return [math.radians(j) for j in joints_deg]
        return None

    def move_to_joints(self, joints_deg: list, wait: bool = True) -> bool:
        """
        Move robot to specified joint positions.

        Args:
            joints_deg: List of 6 joint angles in degrees
            wait: If True, wait for movement to complete

        Returns:
            True if movement successful
        """
        if len(joints_deg) != 6:
            self.get_logger().error(f'Expected 6 joint values, got {len(joints_deg)}')
            return False

        # Format command
        cmd = 'MOVEJ ' + ' '.join([f'{j:.2f}' for j in joints_deg])
        self.get_logger().info(f'Sending: {cmd}')

        # Send command (longer timeout for movement)
        response = self._send_command(cmd, timeout=60.0)

        if 'OK' in response:
            self.get_logger().info(f'Move completed: {response}')
            self._publish_joint_state(joints_deg)
            return True
        else:
            self.get_logger().error(f'Move failed: {response}')
            return False

    def move_to_joints_radians(self, joints_rad: list, wait: bool = True) -> bool:
        """Move robot to specified joint positions in radians."""
        joints_deg = [math.degrees(j) for j in joints_rad]
        return self.move_to_joints(joints_deg, wait)

    def move_home(self) -> bool:
        """Move robot to home position."""
        self.get_logger().info('Moving to home position...')
        return self.move_to_joints(self.HOME_POSITION)

    def set_speed(self, speed_mm_s: int) -> bool:
        """Set robot speed in mm/s (1-500)."""
        if not 1 <= speed_mm_s <= 500:
            self.get_logger().error('Speed must be between 1 and 500 mm/s')
            return False

        response = self._send_command(f'SPEED {speed_mm_s}')
        if 'OK' in response:
            self.get_logger().info(f'Speed set to {speed_mm_s} mm/s')
            return True
        else:
            self.get_logger().error(f'Failed to set speed: {response}')
            return False

    def stop(self) -> bool:
        """Stop robot movement."""
        response = self._send_command('STOP')
        self.get_logger().info(f'Stop command sent: {response}')
        return 'OK' in response

    def execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        """
        Execute a JointTrajectory on the real robot.

        Args:
            trajectory: ROS JointTrajectory message

        Returns:
            True if all waypoints executed successfully
        """
        if not trajectory.points:
            self.get_logger().warn('Empty trajectory')
            return True

        self.get_logger().info(f'Executing trajectory with {len(trajectory.points)} points')

        # Map joint names to indices
        joint_indices = []
        for name in self.JOINT_NAMES:
            if name in trajectory.joint_names:
                joint_indices.append(trajectory.joint_names.index(name))
            else:
                self.get_logger().error(f'Joint {name} not found in trajectory')
                return False

        # Execute each waypoint
        for i, point in enumerate(trajectory.points):
            # Extract joint positions in the correct order
            joints_rad = [point.positions[idx] for idx in joint_indices]
            joints_deg = [math.degrees(j) for j in joints_rad]

            self.get_logger().info(f'Waypoint {i+1}/{len(trajectory.points)}: {joints_deg}')

            if not self.move_to_joints(joints_deg):
                self.get_logger().error(f'Failed at waypoint {i+1}')
                return False

        self.get_logger().info('Trajectory execution completed')
        return True

    def export_to_rapid(self, trajectory: JointTrajectory,
                        filename: str,
                        module_name: str = "GeneratedProgram",
                        speed: int = 50,
                        zone: str = "z1",
                        linear_tolerance: float = 0.001) -> dict:
        """
        Export a JointTrajectory to RAPID .mod file with linear optimization.

        This converts the trajectory to RAPID code, automatically detecting
        linear segments and using MoveL instead of MoveAbsJ where possible.

        Args:
            trajectory: ROS JointTrajectory message
            filename: Output file path for .mod file
            module_name: Name of the RAPID module
            speed: TCP speed in mm/s (default 50)
            zone: Zone data (z0, z1, z5, z10, z50, z100, fine)
            linear_tolerance: Max deviation for linear detection in meters (default 1mm)

        Returns:
            Dictionary with conversion statistics:
            - total_waypoints: Number of input waypoints
            - linear_waypoints: Number detected as linear
            - movej_commands: Number of MoveAbsJ commands
            - movel_commands: Number of MoveL commands
            - optimization_ratio: How much command count was reduced
        """
        if not trajectory.points:
            self.get_logger().warn('Empty trajectory, nothing to export')
            return {'total_waypoints': 0}

        self.get_logger().info(f'Exporting trajectory with {len(trajectory.points)} points to RAPID')

        # Create converter
        converter = TrajectoryToRAPID()

        # Load trajectory
        try:
            num_points = converter.load_trajectory(trajectory)
            self.get_logger().info(f'Loaded {num_points} waypoints')
        except ValueError as e:
            self.get_logger().error(f'Failed to load trajectory: {e}')
            return {'error': str(e)}

        # Detect linear segments
        linear_count, total_count = converter.detect_linear_segments(tolerance=linear_tolerance)
        self.get_logger().info(f'Detected {linear_count}/{total_count} linear waypoints (tolerance: {linear_tolerance*1000:.1f}mm)')

        # Get statistics before export
        stats = converter.get_statistics()

        # Export to file
        try:
            rapid_code = converter.export_rapid(
                filename=filename,
                module_name=module_name,
                speed=speed,
                zone=zone
            )
            self.get_logger().info(f'RAPID code saved to: {filename}')
            self.get_logger().info(f'  MoveAbsJ commands: {stats["movej_commands"]}')
            self.get_logger().info(f'  MoveL commands: {stats["movel_commands"]}')

        except Exception as e:
            self.get_logger().error(f'Failed to export RAPID: {e}')
            return {'error': str(e)}

        return stats

    def _publish_joint_state(self, joints_deg: list):
        """Publish joint state for RViz visualization."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = [math.radians(j) for j in joints_deg]
        self.joint_state_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description='Execute trajectories on real ABB IRB 120')
    parser.add_argument('--ip', default='192.168.125.1', help='Robot IP address')
    parser.add_argument('--port', type=int, default=5000, help='Robot socket port')
    parser.add_argument('--joints', nargs=6, type=float, metavar='J',
                        help='Target joint positions in degrees (6 values)')
    parser.add_argument('--home', action='store_true', help='Move to home position')
    parser.add_argument('--speed', type=int, help='Set speed in mm/s (1-500)')
    parser.add_argument('--status', action='store_true', help='Just show current position')

    args = parser.parse_args()

    rclpy.init()
    executor = RealRobotExecutor(robot_ip=args.ip, robot_port=args.port)

    try:
        if not executor.connect():
            print('\nFailed to connect to robot.')
            print('Make sure:')
            print('  1. Robot is powered on')
            print('  2. SocketMain is running on FlexPendant')
            print('  3. Robot is in AUTO mode with Motors On')
            print(f'  4. Network cable connected and robot reachable at {args.ip}')
            return 1

        # Set speed if specified
        if args.speed:
            executor.set_speed(args.speed)

        # Execute requested action
        if args.status:
            pos = executor.get_current_position()
            if pos:
                print(f'\nCurrent joint positions (degrees):')
                for i, (name, val) in enumerate(zip(executor.JOINT_NAMES, pos)):
                    print(f'  {name}: {val:.2f}')

        elif args.home:
            print('\nMoving to home position...')
            if executor.move_home():
                print('Done!')
            else:
                print('Failed!')
                return 1

        elif args.joints:
            print(f'\nMoving to joints: {args.joints}')
            if executor.move_to_joints(args.joints):
                print('Done!')
            else:
                print('Failed!')
                return 1

        else:
            # Interactive mode
            print('\n=== Real Robot Executor ===')
            print('Commands:')
            print('  status     - Show current position')
            print('  home       - Move to home position')
            print('  move J1 J2 J3 J4 J5 J6 - Move to joints (degrees)')
            print('  speed N    - Set speed (1-500 mm/s)')
            print('  stop       - Stop movement')
            print('  quit       - Exit')
            print()

            while True:
                try:
                    cmd = input('> ').strip().lower()

                    if not cmd:
                        continue
                    elif cmd == 'quit' or cmd == 'exit':
                        break
                    elif cmd == 'status':
                        executor.get_current_position()
                    elif cmd == 'home':
                        executor.move_home()
                    elif cmd == 'stop':
                        executor.stop()
                    elif cmd.startswith('speed '):
                        try:
                            speed = int(cmd.split()[1])
                            executor.set_speed(speed)
                        except (ValueError, IndexError):
                            print('Usage: speed N (1-500)')
                    elif cmd.startswith('move '):
                        try:
                            joints = [float(x) for x in cmd.split()[1:7]]
                            if len(joints) == 6:
                                executor.move_to_joints(joints)
                            else:
                                print('Usage: move J1 J2 J3 J4 J5 J6')
                        except ValueError:
                            print('Usage: move J1 J2 J3 J4 J5 J6')
                    else:
                        print(f'Unknown command: {cmd}')

                except KeyboardInterrupt:
                    print('\nInterrupted')
                    break
                except EOFError:
                    break

    finally:
        executor.disconnect()
        executor.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    exit(main())
