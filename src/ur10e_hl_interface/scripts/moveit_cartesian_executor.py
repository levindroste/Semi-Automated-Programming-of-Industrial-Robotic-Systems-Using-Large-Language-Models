#!/usr/bin/env python3
"""
MoveIt Cartesian Path Executor for ABB IRB 120

Uses MoveIt to plan Cartesian paths (ensuring valid joint configurations),
then executes them on the real robot via socket streaming with MOVEJ commands.

This approach:
1. Uses MoveIt's compute_cartesian_path() for straight-line TCP motion
2. MoveIt validates all joint positions against robot limits
3. Sends planned joint positions via MOVEJ (guaranteed valid)
4. Detects linear segments to reduce number of waypoints sent

Usage:
    ros2 run ur10e_hl_interface moveit_cartesian_executor.py
"""

import rclpy
from rclpy.node import Node
import socket
import time
import math
import numpy as np
from typing import List, Tuple, Optional

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory

# MoveIt Python API
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class MoveItCartesianExecutor(Node):
    """
    Execute Cartesian movements using MoveIt planning + socket streaming.
    """

    JOINT_NAMES = [
        'irb120_joint_1',
        'irb120_joint_2',
        'irb120_joint_3',
        'irb120_joint_4',
        'irb120_joint_5',
        'irb120_joint_6',
    ]

    def __init__(self, robot_ip='192.168.125.1', robot_port=5000):
        super().__init__('moveit_cartesian_executor')

        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.socket = None
        self.connected = False

        # Initialize MoveIt
        self.get_logger().info('Initializing MoveIt...')
        self.moveit = MoveItPy(node_name="moveit_cartesian_executor_moveit")
        self.planning_component = self.moveit.get_planning_component("irb120_arm")
        self.robot_model = self.moveit.get_robot_model()

        self.get_logger().info('MoveIt initialized!')
        self.get_logger().info(f'Robot IP: {robot_ip}:{robot_port}')

    def connect(self) -> bool:
        """Connect to robot socket server."""
        if self.connected:
            return True

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)
            self.socket.connect((self.robot_ip, self.robot_port))
            self.connected = True
            self.get_logger().info(f'Connected to robot')

            # Test connection
            response = self._send_command('PING')
            if 'PONG' in response:
                self.get_logger().info('Connection verified (PONG received)')
                return True
            return True

        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            return False

    def disconnect(self):
        """Disconnect from robot."""
        if self.socket:
            try:
                self._send_command('QUIT')
            except:
                pass
            self.socket.close()
            self.socket = None
            self.connected = False
            self.get_logger().info('Disconnected')

    def _send_command(self, cmd: str, timeout: float = 60.0) -> str:
        """Send command to robot and get response."""
        if not self.connected or not self.socket:
            raise RuntimeError('Not connected')

        self.socket.settimeout(timeout)
        self.socket.send((cmd + '\n').encode())
        time.sleep(0.1)
        response = self.socket.recv(1024).decode().strip()
        return response

    def get_current_joints(self) -> Optional[List[float]]:
        """Get current joint positions in radians from robot."""
        response = self._send_command('GETPOS')
        if response.startswith('POS:'):
            parts = response[4:].split()
            joints_deg = [float(x) for x in parts]
            joints_rad = [math.radians(j) for j in joints_deg]
            return joints_rad
        return None

    def move_joints(self, joints_rad: List[float], timeout: float = 60.0) -> bool:
        """Move robot to joint positions (radians)."""
        joints_deg = [math.degrees(j) for j in joints_rad]
        cmd = 'MOVEJ ' + ' '.join([f'{j:.2f}' for j in joints_deg])
        response = self._send_command(cmd, timeout=timeout)
        return 'OK' in response

    def plan_cartesian_path(self, waypoints: List[Pose],
                           max_step: float = 0.01,
                           jump_threshold: float = 0.0) -> Optional[JointTrajectory]:
        """
        Plan a Cartesian path through waypoints using MoveIt.

        Args:
            waypoints: List of geometry_msgs/Pose for TCP to pass through
            max_step: Maximum step size in meters (smaller = more waypoints)
            jump_threshold: Maximum allowed joint jump (0 = disabled)

        Returns:
            JointTrajectory with valid joint positions, or None if planning fails
        """
        # Get current robot state
        robot_state = self.planning_component.get_start_state()

        # Compute Cartesian path
        # This returns (trajectory, fraction) where fraction is 0.0-1.0
        trajectory, fraction = self.planning_component.compute_cartesian_path(
            waypoints=waypoints,
            max_step=max_step,
            jump_threshold=jump_threshold
        )

        if fraction < 0.9:
            self.get_logger().warn(f'Only {fraction*100:.1f}% of path is feasible')
            return None

        self.get_logger().info(f'Cartesian path planned: {fraction*100:.1f}% feasible, {len(trajectory.joint_trajectory.points)} waypoints')
        return trajectory.joint_trajectory

    def plan_to_pose(self, target_pose: Pose) -> Optional[JointTrajectory]:
        """
        Plan motion to a target Cartesian pose.

        Args:
            target_pose: Target geometry_msgs/Pose

        Returns:
            JointTrajectory or None if planning fails
        """
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = target_pose

        self.planning_component.set_goal_state(pose_stamped=pose_stamped, pose_link="tool0")

        # Plan
        plan_result = self.planning_component.plan()

        if plan_result:
            trajectory = plan_result.trajectory
            self.get_logger().info(f'Motion planned: {len(trajectory.joint_trajectory.points)} waypoints')
            return trajectory.joint_trajectory
        else:
            self.get_logger().error('Planning failed')
            return None

    def execute_trajectory(self, trajectory: JointTrajectory,
                          optimize: bool = True,
                          linear_tolerance: float = 0.001) -> bool:
        """
        Execute a trajectory on the real robot via socket.

        Args:
            trajectory: JointTrajectory from MoveIt
            optimize: If True, reduce waypoints using linear detection
            linear_tolerance: Tolerance for linear segment detection (meters)

        Returns:
            True if execution successful
        """
        if not trajectory.points:
            self.get_logger().warn('Empty trajectory')
            return True

        points = trajectory.points
        num_original = len(points)

        # Build joint index mapping
        joint_indices = []
        for name in self.JOINT_NAMES:
            if name in trajectory.joint_names:
                joint_indices.append(trajectory.joint_names.index(name))
            else:
                self.get_logger().error(f'Joint {name} not in trajectory')
                return False

        # Extract waypoints
        waypoints = []
        for i, point in enumerate(points):
            joints = [point.positions[idx] for idx in joint_indices]
            waypoints.append(joints)

        # Optimize if requested
        if optimize and len(waypoints) > 2:
            waypoints = self._optimize_waypoints(waypoints, linear_tolerance)
            self.get_logger().info(f'Optimized: {num_original} -> {len(waypoints)} waypoints ({100*(1-len(waypoints)/num_original):.0f}% reduction)')

        # Execute each waypoint
        self.get_logger().info(f'Executing {len(waypoints)} waypoints...')
        for i, joints in enumerate(waypoints):
            self.get_logger().info(f'  [{i+1}/{len(waypoints)}] Moving...')
            if not self.move_joints(joints):
                self.get_logger().error(f'Failed at waypoint {i+1}')
                return False

        self.get_logger().info('Trajectory execution complete!')
        return True

    def _optimize_waypoints(self, waypoints: List[List[float]],
                           tolerance: float = 0.001) -> List[List[float]]:
        """
        Reduce waypoints by keeping only segment endpoints.

        Uses forward kinematics to detect linear Cartesian segments,
        then keeps only start/end of each segment.
        """
        if len(waypoints) <= 2:
            return waypoints

        # Import FK from trajectory_to_rapid
        from trajectory_to_rapid import IRB120ForwardKinematics
        fk = IRB120ForwardKinematics()

        # Compute Cartesian positions for all waypoints
        cartesian = []
        for joints in waypoints:
            pose = fk.compute_fk(joints)
            cartesian.append(np.array([pose.x, pose.y, pose.z]))

        # Find linear segments
        optimized = [waypoints[0]]  # Always keep first
        segment_start = 0

        for i in range(2, len(waypoints)):
            # Check if all points from segment_start to i are on a line
            if not self._points_on_line(cartesian, segment_start, i, tolerance):
                # Add the previous endpoint
                optimized.append(waypoints[i-1])
                segment_start = i - 1

        # Always keep last
        if optimized[-1] != waypoints[-1]:
            optimized.append(waypoints[-1])

        return optimized

    def _points_on_line(self, points: List[np.ndarray],
                       start: int, end: int, tolerance: float) -> bool:
        """Check if all points between start and end lie on a line."""
        if end - start < 2:
            return True

        start_pt = points[start]
        end_pt = points[end]
        line_vec = end_pt - start_pt
        line_len = np.linalg.norm(line_vec)

        if line_len < 1e-9:
            return True

        line_dir = line_vec / line_len

        for i in range(start + 1, end):
            pt_vec = points[i] - start_pt
            proj_len = np.dot(pt_vec, line_dir)
            proj_pt = start_pt + proj_len * line_dir
            dist = np.linalg.norm(points[i] - proj_pt)
            if dist > tolerance:
                return False

        return True

    def move_cartesian_relative(self, dx: float = 0, dy: float = 0, dz: float = 0,
                                keep_orientation: bool = True) -> bool:
        """
        Move TCP by relative offset using MoveIt Cartesian planning.

        Args:
            dx, dy, dz: Relative movement in meters
            keep_orientation: Keep current orientation

        Returns:
            True if successful
        """
        # Get current pose from MoveIt
        robot_state = self.planning_component.get_start_state()
        current_pose = robot_state.get_pose("tool0")

        self.get_logger().info(f'Current TCP: ({current_pose.position.x*1000:.1f}, {current_pose.position.y*1000:.1f}, {current_pose.position.z*1000:.1f}) mm')

        # Create target pose
        target = Pose()
        target.position.x = current_pose.position.x + dx
        target.position.y = current_pose.position.y + dy
        target.position.z = current_pose.position.z + dz

        if keep_orientation:
            target.orientation = current_pose.orientation
        else:
            # Default orientation (pointing down)
            target.orientation.w = 0.707107
            target.orientation.x = 0.0
            target.orientation.y = 0.707107
            target.orientation.z = 0.0

        self.get_logger().info(f'Target TCP: ({target.position.x*1000:.1f}, {target.position.y*1000:.1f}, {target.position.z*1000:.1f}) mm')

        # Plan Cartesian path
        trajectory = self.plan_cartesian_path([target], max_step=0.005)

        if trajectory is None:
            self.get_logger().error('Failed to plan Cartesian path')
            return False

        # Execute
        return self.execute_trajectory(trajectory, optimize=True)


def main():
    rclpy.init()

    executor = MoveItCartesianExecutor()

    try:
        if not executor.connect():
            print('Failed to connect to robot')
            return 1

        print('\n=== MoveIt Cartesian Executor ===')
        print('This uses MoveIt to plan valid Cartesian paths')
        print('Commands:')
        print('  down N    - Move N cm down (Z-)')
        print('  up N      - Move N cm up (Z+)')
        print('  left N    - Move N cm left (Y-)')
        print('  right N   - Move N cm right (Y+)')
        print('  forward N - Move N cm forward (X+)')
        print('  back N    - Move N cm back (X-)')
        print('  home      - Move to home position')
        print('  quit      - Exit')
        print()

        while rclpy.ok():
            try:
                cmd = input('> ').strip().lower()

                if not cmd:
                    continue
                elif cmd in ['quit', 'exit']:
                    break
                elif cmd == 'home':
                    executor.move_joints([0, 0, 0, 0, 0, 0])
                elif cmd.startswith('down '):
                    cm = float(cmd.split()[1])
                    executor.move_cartesian_relative(dz=-cm/100)
                elif cmd.startswith('up '):
                    cm = float(cmd.split()[1])
                    executor.move_cartesian_relative(dz=cm/100)
                elif cmd.startswith('left '):
                    cm = float(cmd.split()[1])
                    executor.move_cartesian_relative(dy=-cm/100)
                elif cmd.startswith('right '):
                    cm = float(cmd.split()[1])
                    executor.move_cartesian_relative(dy=cm/100)
                elif cmd.startswith('forward '):
                    cm = float(cmd.split()[1])
                    executor.move_cartesian_relative(dx=cm/100)
                elif cmd.startswith('back '):
                    cm = float(cmd.split()[1])
                    executor.move_cartesian_relative(dx=-cm/100)
                else:
                    print(f'Unknown command: {cmd}')

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f'Error: {e}')

    finally:
        executor.disconnect()
        executor.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    exit(main())
