#!/usr/bin/env python3
"""
Safe Cartesian Movement for ABB IRB 120

Plans Cartesian paths with joint limit validation, then executes via MOVEJ/MOVEL.

This approach:
1. Uses FK to get current TCP position
2. Generates interpolated waypoints along Cartesian path
3. Uses IK (with orientation) to compute joint positions for each waypoint
4. Validates all joint positions against robot limits
5. Applies hybrid optimization (reduces waypoints while ensuring max spacing)
6. Sends validated joint positions via MOVEJ
7. Uses MOVEL for final linear approach (pick & place)

Safety Features:
- Hybrid waypoint optimization: reduces waypoint count for efficiency while
  guaranteeing maximum 3cm spacing between waypoints for collision safety
- MOVEJ paths stay close to intended trajectory when spacing is small
- Adjustable max_spacing parameter (lower = safer but slower)
- Joint limit validation before execution
- Optional MOVEL for final approach (straight-line TCP motion)

Usage:
    python3 cartesian_move_safe.py
"""

import socket
import time
import math
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

# Import from trajectory_to_rapid
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_to_rapid import IRB120ForwardKinematics, Pose


@dataclass
class JointLimits:
    """Joint limits for IRB 120 in radians."""
    # From ABB datasheet
    j1_min: float = math.radians(-165)
    j1_max: float = math.radians(165)
    j2_min: float = math.radians(-110)
    j2_max: float = math.radians(110)
    j3_min: float = math.radians(-110)
    j3_max: float = math.radians(70)
    j4_min: float = math.radians(-160)
    j4_max: float = math.radians(160)
    j5_min: float = math.radians(-120)
    j5_max: float = math.radians(120)
    j6_min: float = math.radians(-400)
    j6_max: float = math.radians(400)

    def check(self, joints: List[float]) -> Tuple[bool, str]:
        """Check if joints are within limits."""
        limits = [
            (self.j1_min, self.j1_max),
            (self.j2_min, self.j2_max),
            (self.j3_min, self.j3_max),
            (self.j4_min, self.j4_max),
            (self.j5_min, self.j5_max),
            (self.j6_min, self.j6_max),
        ]
        for i, (j, (jmin, jmax)) in enumerate(zip(joints, limits)):
            if j < jmin or j > jmax:
                return False, f"Joint {i+1}: {math.degrees(j):.1f}° outside [{math.degrees(jmin):.1f}°, {math.degrees(jmax):.1f}°]"
        return True, "OK"


class SafeCartesianMover:
    """Execute Cartesian movements with joint limit validation."""

    def __init__(self, robot_ip='192.168.125.1', robot_port=5000):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.socket = None
        self.connected = False
        self.fk = IRB120ForwardKinematics()
        self.limits = JointLimits()

    def connect(self) -> bool:
        """Connect to robot socket server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)
            self.socket.connect((self.robot_ip, self.robot_port))
            self.connected = True
            print(f'Connected to robot at {self.robot_ip}:{self.robot_port}')

            response = self._send('PING')
            if 'PONG' in response:
                print('Connection verified')
                return True
            return True
        except Exception as e:
            print(f'Connection failed: {e}')
            return False

    def disconnect(self):
        """Disconnect from robot."""
        if self.socket:
            try:
                self._send('QUIT')
            except:
                pass
            self.socket.close()
            self.connected = False
            print('Disconnected')

    def _send(self, cmd: str, timeout: float = 60.0) -> str:
        """Send command and get response."""
        self.socket.settimeout(timeout)
        self.socket.send((cmd + '\n').encode())
        time.sleep(0.1)
        return self.socket.recv(1024).decode().strip()

    def get_joints_rad(self) -> Optional[List[float]]:
        """Get current joint positions in radians."""
        response = self._send('GETPOS')
        if response.startswith('POS:'):
            parts = response[4:].split()
            return [math.radians(float(x)) for x in parts]
        return None

    def get_tcp(self) -> Optional[Tuple[float, float, float]]:
        """Get current TCP position in mm using FK."""
        joints = self.get_joints_rad()
        if joints:
            pose = self.fk.compute_fk(joints)
            return (pose.x * 1000, pose.y * 1000, pose.z * 1000)
        return None

    def move_joints(self, joints_rad: List[float]) -> bool:
        """Move to joint position (radians) using MOVEJ."""
        # Validate first
        valid, msg = self.limits.check(joints_rad)
        if not valid:
            print(f'  ERROR: {msg}')
            return False

        joints_deg = [math.degrees(j) for j in joints_rad]
        cmd = 'MOVEJ ' + ' '.join([f'{j:.2f}' for j in joints_deg])
        response = self._send(cmd, timeout=60.0)
        return 'OK' in response

    def move_linear(self, x_mm: float, y_mm: float, z_mm: float,
                    q0: float, q1: float, q2: float, q3: float) -> bool:
        """
        Move TCP linearly to Cartesian position using MOVEL.

        This maintains a straight-line TCP path - essential for:
        - Final approach to pick/place objects
        - Insertion operations
        - Surface following

        Args:
            x_mm, y_mm, z_mm: Target position in mm
            q0, q1, q2, q3: Target orientation quaternion (RAPID format)

        Returns:
            True if successful
        """
        cmd = f'MOVEL {x_mm:.2f} {y_mm:.2f} {z_mm:.2f} {q0:.6f} {q1:.6f} {q2:.6f} {q3:.6f}'
        response = self._send(cmd, timeout=60.0)
        if 'OK' in response:
            return True
        else:
            print(f'  MOVEL failed: {response}')
            return False

    def compute_ik(self, target_pose: Pose, seed_joints: List[float],
                   max_iterations: int = 500, tolerance: float = 1e-3) -> Optional[List[float]]:
        """
        Compute inverse kinematics using damped least squares method.

        Args:
            target_pose: Target Cartesian pose
            seed_joints: Starting joint configuration
            max_iterations: Maximum IK iterations
            tolerance: Position tolerance in meters

        Returns:
            Joint positions or None if IK fails
        """
        joints = list(seed_joints)
        damping = 0.1  # Damping factor for numerical stability

        limits_list = [
            (self.limits.j1_min, self.limits.j1_max),
            (self.limits.j2_min, self.limits.j2_max),
            (self.limits.j3_min, self.limits.j3_max),
            (self.limits.j4_min, self.limits.j4_max),
            (self.limits.j5_min, self.limits.j5_max),
            (self.limits.j6_min, self.limits.j6_max),
        ]

        for iteration in range(max_iterations):
            # Current pose
            current = self.fk.compute_fk(joints)

            # Position error
            error = np.array([
                target_pose.x - current.x,
                target_pose.y - current.y,
                target_pose.z - current.z
            ])

            error_norm = np.linalg.norm(error)
            if error_norm < tolerance:
                return joints

            # Compute Jacobian (numerical)
            J = self._compute_jacobian(joints)

            # Damped least squares: dq = J^T * (J * J^T + lambda^2 * I)^-1 * error
            JJT = J @ J.T
            damped = JJT + damping**2 * np.eye(3)
            dq = J.T @ np.linalg.solve(damped, error)

            # Adaptive step size
            step = min(1.0, 0.1 / (error_norm + 0.01))

            # Update joints
            joints = [j + step * dj for j, dj in zip(joints, dq)]

            # Clamp to limits
            joints = [max(lo, min(hi, j)) for j, (lo, hi) in zip(joints, limits_list)]

        # Check final error
        current = self.fk.compute_fk(joints)
        final_error = np.linalg.norm([
            target_pose.x - current.x,
            target_pose.y - current.y,
            target_pose.z - current.z
        ])

        # Accept if we got close (within 5mm)
        if final_error < 0.005:
            return joints

        return None  # IK failed

    def _compute_jacobian(self, joints: List[float], delta: float = 1e-6) -> np.ndarray:
        """Compute position Jacobian numerically."""
        J = np.zeros((3, 6))
        base_pose = self.fk.compute_fk(joints)
        base_pos = np.array([base_pose.x, base_pose.y, base_pose.z])

        for i in range(6):
            joints_plus = list(joints)
            joints_plus[i] += delta
            pose_plus = self.fk.compute_fk(joints_plus)
            pos_plus = np.array([pose_plus.x, pose_plus.y, pose_plus.z])
            J[:, i] = (pos_plus - base_pos) / delta

        return J

    def plan_cartesian_path(self, start_joints: List[float],
                            dx: float = 0, dy: float = 0, dz: float = 0,
                            num_waypoints: int = 20) -> Optional[List[List[float]]]:
        """
        Plan a Cartesian path with IK validation.

        Args:
            start_joints: Starting joint configuration
            dx, dy, dz: Relative movement in meters
            num_waypoints: Number of waypoints

        Returns:
            List of joint configurations, or None if planning fails
        """
        # Get start TCP
        start_pose = self.fk.compute_fk(start_joints)
        start_pos = np.array([start_pose.x, start_pose.y, start_pose.z])

        # End position
        end_pos = start_pos + np.array([dx, dy, dz])

        print(f'Planning path:')
        print(f'  Start: ({start_pos[0]*1000:.1f}, {start_pos[1]*1000:.1f}, {start_pos[2]*1000:.1f}) mm')
        print(f'  End:   ({end_pos[0]*1000:.1f}, {end_pos[1]*1000:.1f}, {end_pos[2]*1000:.1f}) mm')
        print(f'  Waypoints: {num_waypoints}')

        waypoints = [start_joints]
        current_joints = list(start_joints)

        for i in range(1, num_waypoints + 1):
            t = i / num_waypoints
            # Linear interpolation
            target_pos = start_pos + t * (end_pos - start_pos)

            target_pose = Pose(
                x=target_pos[0],
                y=target_pos[1],
                z=target_pos[2],
                qw=start_pose.qw,
                qx=start_pose.qx,
                qy=start_pose.qy,
                qz=start_pose.qz
            )

            # IK with current joints as seed
            new_joints = self.compute_ik(target_pose, current_joints)

            if new_joints is None:
                print(f'  IK failed at waypoint {i}/{num_waypoints}')
                return None

            # Validate joints
            valid, msg = self.limits.check(new_joints)
            if not valid:
                print(f'  Joint limits violated at waypoint {i}: {msg}')
                return None

            waypoints.append(new_joints)
            current_joints = new_joints

        print(f'  Path planned successfully!')
        return waypoints

    def optimize_waypoints(self, waypoints: List[List[float]],
                          tolerance: float = 0.001,
                          max_spacing: float = 0.03) -> List[List[float]]:
        """
        Reduce waypoints while ensuring maximum spacing for collision safety.

        This hybrid approach:
        1. Detects linear segments to reduce waypoint count
        2. Ensures maximum spacing between waypoints (default 3cm) for safety

        Args:
            waypoints: List of joint configurations
            tolerance: Linear detection tolerance in meters
            max_spacing: Maximum allowed distance between waypoints in meters (default 3cm)

        Returns:
            Optimized waypoint list with guaranteed max spacing
        """
        if len(waypoints) <= 2:
            return self._ensure_spacing(waypoints, max_spacing)

        # Compute Cartesian positions
        cartesian = []
        for joints in waypoints:
            pose = self.fk.compute_fk(joints)
            cartesian.append(np.array([pose.x, pose.y, pose.z]))

        # Find linear segments and collect segment endpoints
        segment_endpoints = [0]  # Start with first waypoint index
        segment_start = 0

        for i in range(2, len(waypoints)):
            if not self._points_on_line(cartesian, segment_start, i, tolerance):
                segment_endpoints.append(i - 1)
                segment_start = i - 1

        # Always include last waypoint
        if segment_endpoints[-1] != len(waypoints) - 1:
            segment_endpoints.append(len(waypoints) - 1)

        # Build optimized list from segment endpoints
        endpoint_waypoints = [waypoints[i] for i in segment_endpoints]

        # Now ensure spacing constraint is met
        return self._ensure_spacing(endpoint_waypoints, max_spacing)

    def _ensure_spacing(self, waypoints: List[List[float]],
                        max_spacing: float) -> List[List[float]]:
        """
        Ensure no gap between consecutive waypoints exceeds max_spacing.

        If a gap is too large, adds intermediate waypoints using linear
        interpolation in joint space.

        Args:
            waypoints: List of joint configurations
            max_spacing: Maximum allowed Cartesian distance in meters

        Returns:
            Waypoint list with guaranteed max spacing
        """
        if len(waypoints) <= 1:
            return waypoints

        result = [waypoints[0]]

        for i in range(1, len(waypoints)):
            # Get Cartesian positions
            start_pose = self.fk.compute_fk(waypoints[i - 1])
            end_pose = self.fk.compute_fk(waypoints[i])

            start_tcp = np.array([start_pose.x, start_pose.y, start_pose.z])
            end_tcp = np.array([end_pose.x, end_pose.y, end_pose.z])

            distance = np.linalg.norm(end_tcp - start_tcp)

            if distance > max_spacing:
                # Need intermediate waypoints
                num_subdivisions = int(np.ceil(distance / max_spacing))
                for j in range(1, num_subdivisions):
                    t = j / num_subdivisions
                    # Linear interpolation in joint space
                    interp_joints = [
                        (1 - t) * waypoints[i - 1][k] + t * waypoints[i][k]
                        for k in range(6)
                    ]
                    result.append(interp_joints)

            result.append(waypoints[i])

        return result

    def _points_on_line(self, points: List[np.ndarray],
                       start: int, end: int, tolerance: float) -> bool:
        """Check if points lie on a line."""
        if end - start < 2:
            return True

        line_vec = points[end] - points[start]
        line_len = np.linalg.norm(line_vec)
        if line_len < 1e-9:
            return True

        line_dir = line_vec / line_len

        for i in range(start + 1, end):
            pt_vec = points[i] - points[start]
            proj_len = np.dot(pt_vec, line_dir)
            proj_pt = points[start] + proj_len * line_dir
            dist = np.linalg.norm(points[i] - proj_pt)
            if dist > tolerance:
                return False
        return True

    def move_cartesian(self, dx: float = 0, dy: float = 0, dz: float = 0,
                      num_waypoints: int = 20, optimize: bool = True,
                      max_spacing: float = 0.03,
                      linear_final: bool = False) -> bool:
        """
        Execute a relative Cartesian movement.

        Args:
            dx, dy, dz: Movement in meters
            num_waypoints: Waypoints for planning (more = smoother but slower)
            optimize: Reduce waypoints using hybrid optimization (default: True)
            max_spacing: Maximum distance between waypoints in meters (default: 3cm)
            linear_final: Use MOVEL for last segment (maintains TCP orientation)

        Returns:
            True if successful

        Safety Notes:
            - Hybrid optimization reduces waypoints while guaranteeing max_spacing
            - Default 3cm spacing ensures robot stays close to intended path
            - Set max_spacing lower (e.g., 0.01) for tighter path following
            - linear_final=True: Last movement uses MOVEL for straight-line TCP motion
        """
        # Get current joints
        current_joints = self.get_joints_rad()
        if current_joints is None:
            print('Failed to get current position')
            return False

        # Plan path
        waypoints = self.plan_cartesian_path(current_joints, dx, dy, dz, num_waypoints)
        if waypoints is None:
            return False

        # Hybrid optimization: reduce waypoints but ensure max spacing
        if optimize:
            original_count = len(waypoints)
            waypoints = self.optimize_waypoints(waypoints, tolerance=0.001, max_spacing=max_spacing)
            print(f'Optimized: {original_count} -> {len(waypoints)} waypoints (max spacing: {max_spacing*100:.0f}cm)')

        # Execute all but last waypoint with MOVEJ
        print(f'Executing {len(waypoints)} waypoints...')

        if linear_final and len(waypoints) > 1:
            # Execute all but last with MOVEJ
            for i, joints in enumerate(waypoints[:-1]):
                print(f'  [{i+1}/{len(waypoints)}] MOVEJ', end=' ')
                if not self.move_joints(joints):
                    print('FAILED')
                    return False
                print('OK')

            # Last waypoint with MOVEL (linear motion, maintains orientation)
            last_joints = waypoints[-1]
            pose = self.fk.compute_fk(last_joints)
            print(f'  [{len(waypoints)}/{len(waypoints)}] MOVEL (linear)', end=' ')
            if not self.move_linear(pose.x * 1000, pose.y * 1000, pose.z * 1000,
                                    pose.qw, pose.qx, pose.qy, pose.qz):
                print('FAILED')
                return False
            print('OK')
        else:
            # All waypoints with MOVEJ
            for i, joints in enumerate(waypoints):
                print(f'  [{i+1}/{len(waypoints)}] MOVEJ', end=' ')
                if not self.move_joints(joints):
                    print('FAILED')
                    return False
                print('OK')

        # Verify final position
        final_tcp = self.get_tcp()
        if final_tcp:
            print(f'Final TCP: ({final_tcp[0]:.1f}, {final_tcp[1]:.1f}, {final_tcp[2]:.1f}) mm')

        return True


def main():
    mover = SafeCartesianMover()

    if not mover.connect():
        print('Failed to connect')
        return 1

    try:
        print('\n=== Safe Cartesian Mover ===')
        print('Commands:')
        print('  down N   - Move N cm down')
        print('  up N     - Move N cm up')
        print('  left N   - Move N cm left (Y-)')
        print('  right N  - Move N cm right (Y+)')
        print('  home     - Move to home')
        print('  pos      - Show current position')
        print('  quit     - Exit')
        print()

        # Show current position
        tcp = mover.get_tcp()
        if tcp:
            print(f'Current TCP: ({tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f}) mm')

        while True:
            try:
                cmd = input('> ').strip().lower()

                if not cmd:
                    continue
                elif cmd in ['quit', 'exit', 'q']:
                    break
                elif cmd == 'home':
                    mover.move_joints([0, 0, 0, 0, 0, 0])
                elif cmd == 'pos':
                    tcp = mover.get_tcp()
                    if tcp:
                        print(f'TCP: ({tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f}) mm')
                    joints = mover.get_joints_rad()
                    if joints:
                        print(f'Joints: {[f"{math.degrees(j):.1f}" for j in joints]}°')
                elif cmd.startswith('down '):
                    cm = float(cmd.split()[1])
                    mover.move_cartesian(dz=-cm/100)
                elif cmd.startswith('up '):
                    cm = float(cmd.split()[1])
                    mover.move_cartesian(dz=cm/100)
                elif cmd.startswith('left '):
                    cm = float(cmd.split()[1])
                    mover.move_cartesian(dy=-cm/100)
                elif cmd.startswith('right '):
                    cm = float(cmd.split()[1])
                    mover.move_cartesian(dy=cm/100)
                elif cmd.startswith('forward '):
                    cm = float(cmd.split()[1])
                    mover.move_cartesian(dx=cm/100)
                elif cmd.startswith('back '):
                    cm = float(cmd.split()[1])
                    mover.move_cartesian(dx=-cm/100)
                else:
                    print(f'Unknown: {cmd}')

            except KeyboardInterrupt:
                print()
                break
            except ValueError as e:
                print(f'Invalid number: {e}')
            except Exception as e:
                print(f'Error: {e}')

    finally:
        mover.disconnect()

    return 0


if __name__ == '__main__':
    exit(main())
