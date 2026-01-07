#!/usr/bin/env python3
"""
MoveIt Trajectory to RAPID Code Converter

Converts ROS2 JointTrajectory messages into optimized RAPID .mod files
with intelligent linear movement (MoveL) detection.

Architecture:
    JointTrajectory → Forward Kinematics → Cartesian Waypoints
                   → Linear Segment Detector → Optimized Commands
                   → RAPID Code Generator → .mod file

Usage:
    from trajectory_to_rapid import TrajectoryToRAPID

    converter = TrajectoryToRAPID()
    converter.load_trajectory(joint_trajectory_msg)
    converter.detect_linear_segments(tolerance=0.001)  # 1mm
    converter.export_rapid("output.mod")
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Union
from enum import Enum


class MoveType(Enum):
    """Type of robot movement command."""
    MOVEJ = "MoveAbsJ"  # Joint space movement
    MOVEL = "MoveL"      # Linear Cartesian movement


@dataclass
class Pose:
    """Cartesian pose with position and orientation (quaternion)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @property
    def quaternion(self) -> np.ndarray:
        """Return quaternion as [qw, qx, qy, qz] (RAPID format)."""
        return np.array([self.qw, self.qx, self.qy, self.qz])

    def distance_to(self, other: 'Pose') -> float:
        """Calculate Euclidean distance to another pose."""
        return np.linalg.norm(self.position - other.position)

    def to_rapid_robtarget(self) -> str:
        """Convert to RAPID robtarget format."""
        # Position in mm (RAPID uses mm, we use meters internally)
        x_mm = self.x * 1000
        y_mm = self.y * 1000
        z_mm = self.z * 1000

        # RAPID robtarget: [[x,y,z], [q1,q2,q3,q4], [cf1,cf4,cf6,cfx], [eax_a,...]]
        # q1=qw, q2=qx, q3=qy, q4=qz in RAPID
        return f"[[{x_mm:.2f},{y_mm:.2f},{z_mm:.2f}],[{self.qw:.6f},{self.qx:.6f},{self.qy:.6f},{self.qz:.6f}],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]"


@dataclass
class Waypoint:
    """A single waypoint in the trajectory."""
    joint_positions: List[float]  # radians
    cartesian_pose: Optional[Pose] = None
    time_from_start: float = 0.0
    index: int = 0

    # Set by linear detector
    is_linear: bool = False
    segment_id: int = -1
    move_type: MoveType = MoveType.MOVEJ

    @property
    def joints_degrees(self) -> List[float]:
        """Return joint positions in degrees."""
        return [math.degrees(j) for j in self.joint_positions]

    def to_rapid_jointtarget(self) -> str:
        """Convert to RAPID jointtarget format."""
        joints_deg = self.joints_degrees
        # RAPID jointtarget: [[j1,j2,j3,j4,j5,j6], [eax_a,...]]
        return f"[[{joints_deg[0]:.4f},{joints_deg[1]:.4f},{joints_deg[2]:.4f},{joints_deg[3]:.4f},{joints_deg[4]:.4f},{joints_deg[5]:.4f}],[9E9,9E9,9E9,9E9,9E9,9E9]]"


@dataclass
class LinearSegment:
    """A detected linear segment in the trajectory."""
    start_idx: int
    end_idx: int
    waypoints: List[Waypoint] = field(default_factory=list)

    @property
    def length(self) -> int:
        return self.end_idx - self.start_idx + 1

    @property
    def is_significant(self) -> bool:
        """Check if segment is worth using MoveL (more than 2 points)."""
        return self.length > 2


class IRB120ForwardKinematics:
    """
    Forward kinematics for ABB IRB 120 robot.

    DH parameters extracted from URDF:
    - Joint 1: Z-axis rotation at base
    - Joint 2: Y-axis rotation at (0, 0, 0.29)
    - Joint 3: Y-axis rotation at (0, 0, 0.27)
    - Joint 4: X-axis rotation at (0, 0, 0.07)
    - Joint 5: Y-axis rotation at (0.302, 0, 0)
    - Joint 6: X-axis rotation at (0.072, 0, 0)
    """

    # Link lengths from URDF (meters)
    L1 = 0.29   # base to joint 2
    L2 = 0.27   # joint 2 to joint 3
    L3 = 0.07   # joint 3 to joint 4
    L4 = 0.302  # joint 4 to joint 5
    L5 = 0.072  # joint 5 to joint 6 (flange)

    @staticmethod
    def rotation_x(angle: float) -> np.ndarray:
        """Rotation matrix around X-axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])

    @staticmethod
    def rotation_y(angle: float) -> np.ndarray:
        """Rotation matrix around Y-axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, 0, s],
            [0, 1, 0],
            [-s, 0, c]
        ])

    @staticmethod
    def rotation_z(angle: float) -> np.ndarray:
        """Rotation matrix around Z-axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])

    @staticmethod
    def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
        """
        Convert rotation matrix to quaternion (qw, qx, qy, qz).

        Uses Shepperd's method for numerical stability.
        """
        trace = R[0, 0] + R[1, 1] + R[2, 2]

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        # Normalize
        norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        return qw/norm, qx/norm, qy/norm, qz/norm

    def compute_fk(self, joints: List[float]) -> Pose:
        """
        Compute forward kinematics for given joint angles.

        Args:
            joints: List of 6 joint angles in radians

        Returns:
            Pose of the tool0 frame
        """
        j1, j2, j3, j4, j5, j6 = joints

        # Build transformation step by step
        # Start at base
        T = np.eye(4)

        # Joint 1: Rotate around Z at base
        R1 = self.rotation_z(j1)
        T[:3, :3] = R1

        # Move up to joint 2
        T[:3, 3] = T[:3, :3] @ np.array([0, 0, self.L1])

        # Joint 2: Rotate around Y
        R2 = self.rotation_y(j2)
        T[:3, :3] = T[:3, :3] @ R2

        # Move to joint 3
        T[:3, 3] += T[:3, :3] @ np.array([0, 0, self.L2])

        # Joint 3: Rotate around Y
        R3 = self.rotation_y(j3)
        T[:3, :3] = T[:3, :3] @ R3

        # Move to joint 4
        T[:3, 3] += T[:3, :3] @ np.array([0, 0, self.L3])

        # Joint 4: Rotate around X
        R4 = self.rotation_x(j4)
        T[:3, :3] = T[:3, :3] @ R4

        # Move to joint 5
        T[:3, 3] += T[:3, :3] @ np.array([self.L4, 0, 0])

        # Joint 5: Rotate around Y
        R5 = self.rotation_y(j5)
        T[:3, :3] = T[:3, :3] @ R5

        # Move to joint 6 (flange)
        T[:3, 3] += T[:3, :3] @ np.array([self.L5, 0, 0])

        # Joint 6: Rotate around X
        R6 = self.rotation_x(j6)
        T[:3, :3] = T[:3, :3] @ R6

        # Apply tool0 frame rotation (pi/2 around Y from URDF)
        R_tool0 = self.rotation_y(np.pi / 2)
        T[:3, :3] = T[:3, :3] @ R_tool0

        # Extract position and orientation
        position = T[:3, 3]
        qw, qx, qy, qz = self.rotation_matrix_to_quaternion(T[:3, :3])

        return Pose(
            x=position[0],
            y=position[1],
            z=position[2],
            qw=qw, qx=qx, qy=qy, qz=qz
        )


class LinearSegmentDetector:
    """
    Detect linear segments in trajectory for MoveL optimization.

    Algorithm:
    1. Convert all waypoints to Cartesian poses using FK
    2. For consecutive waypoints, check if they form a line
    3. Group linear waypoints into segments
    4. Non-linear transitions use MoveJ
    """

    def __init__(self, tolerance: float = 0.001):
        """
        Args:
            tolerance: Maximum perpendicular distance in meters (default 1mm)
        """
        self.tolerance = tolerance
        self.fk = IRB120ForwardKinematics()

    def compute_cartesian_poses(self, waypoints: List[Waypoint]) -> None:
        """Compute Cartesian pose for each waypoint using FK."""
        for wp in waypoints:
            wp.cartesian_pose = self.fk.compute_fk(wp.joint_positions)

    def perpendicular_distance(self, point: np.ndarray,
                                line_start: np.ndarray,
                                line_end: np.ndarray) -> float:
        """
        Calculate perpendicular distance from point to line segment.

        Args:
            point: The point to measure distance from
            line_start: Start of line segment
            line_end: End of line segment

        Returns:
            Perpendicular distance in meters
        """
        line_vec = line_end - line_start
        line_len = np.linalg.norm(line_vec)

        if line_len < 1e-10:
            # Line segment is a point
            return np.linalg.norm(point - line_start)

        line_unit = line_vec / line_len

        # Project point onto line
        point_vec = point - line_start
        proj_length = np.dot(point_vec, line_unit)

        # Clamp to line segment
        proj_length = max(0, min(line_len, proj_length))

        # Closest point on line
        closest = line_start + proj_length * line_unit

        return np.linalg.norm(point - closest)

    def points_are_collinear(self, waypoints: List[Waypoint],
                             start_idx: int, end_idx: int) -> bool:
        """
        Check if all intermediate points lie on line from start to end.

        Args:
            waypoints: List of all waypoints
            start_idx: Index of segment start
            end_idx: Index of segment end

        Returns:
            True if all intermediate points are within tolerance of line
        """
        if end_idx - start_idx < 2:
            return True  # 2 points always form a line

        start_pos = waypoints[start_idx].cartesian_pose.position
        end_pos = waypoints[end_idx].cartesian_pose.position

        for i in range(start_idx + 1, end_idx):
            point = waypoints[i].cartesian_pose.position
            dist = self.perpendicular_distance(point, start_pos, end_pos)
            if dist > self.tolerance:
                return False

        return True

    def detect_segments(self, waypoints: List[Waypoint]) -> List[LinearSegment]:
        """
        Detect linear segments in trajectory.

        Args:
            waypoints: List of waypoints with Cartesian poses computed

        Returns:
            List of LinearSegment objects
        """
        if len(waypoints) < 2:
            return []

        # Ensure Cartesian poses are computed
        if waypoints[0].cartesian_pose is None:
            self.compute_cartesian_poses(waypoints)

        segments = []
        segment_start = 0
        current_segment_id = 0

        i = 1
        while i < len(waypoints):
            # Try to extend current segment
            if self.points_are_collinear(waypoints, segment_start, i):
                # Can extend, continue
                i += 1
            else:
                # Cannot extend, close segment
                if i - 1 > segment_start:
                    # We had a valid segment ending at i-1
                    segment = LinearSegment(
                        start_idx=segment_start,
                        end_idx=i - 1,
                        waypoints=waypoints[segment_start:i]
                    )

                    # Mark waypoints as linear
                    for wp in segment.waypoints:
                        wp.is_linear = segment.is_significant
                        wp.segment_id = current_segment_id
                        wp.move_type = MoveType.MOVEL if segment.is_significant else MoveType.MOVEJ

                    segments.append(segment)
                    current_segment_id += 1

                # Start new segment from i-1
                segment_start = i - 1
                i += 1

        # Don't forget the last segment
        if segment_start < len(waypoints) - 1:
            segment = LinearSegment(
                start_idx=segment_start,
                end_idx=len(waypoints) - 1,
                waypoints=waypoints[segment_start:]
            )

            for wp in segment.waypoints:
                wp.is_linear = segment.is_significant
                wp.segment_id = current_segment_id
                wp.move_type = MoveType.MOVEL if segment.is_significant else MoveType.MOVEJ

            segments.append(segment)

        return segments


class RAPIDGenerator:
    """
    Generate RAPID .mod files from optimized trajectory.
    """

    def __init__(self, module_name: str = "GeneratedProgram",
                 speed_tcp: int = 100,
                 speed_ori: int = 50,
                 zone: str = "z1"):
        """
        Args:
            module_name: Name of the RAPID module
            speed_tcp: TCP speed in mm/s
            speed_ori: Orientation speed in deg/s
            zone: Zone data (z0, z1, z5, z10, z50, z100, fine)
        """
        self.module_name = module_name
        self.speed_tcp = speed_tcp
        self.speed_ori = speed_ori
        self.zone = zone

        self.targets: List[str] = []
        self.instructions: List[str] = []
        self.target_counter = 0

    def add_joint_target(self, waypoint: Waypoint) -> str:
        """Add a joint target and return its name."""
        self.target_counter += 1
        name = f"jTarget{self.target_counter}"
        target_def = f"    CONST jointtarget {name} := {waypoint.to_rapid_jointtarget()};"
        self.targets.append(target_def)
        return name

    def add_rob_target(self, waypoint: Waypoint) -> str:
        """Add a robtarget and return its name."""
        self.target_counter += 1
        name = f"pTarget{self.target_counter}"
        target_def = f"    CONST robtarget {name} := {waypoint.cartesian_pose.to_rapid_robtarget()};"
        self.targets.append(target_def)
        return name

    def add_movej(self, target_name: str):
        """Add MoveAbsJ instruction."""
        self.instructions.append(
            f"        MoveAbsJ {target_name}, v{self.speed_tcp}, {self.zone}, tool0;"
        )

    def add_movel(self, target_name: str):
        """Add MoveL instruction."""
        self.instructions.append(
            f"        MoveL {target_name}, v{self.speed_tcp}, {self.zone}, tool0;"
        )

    def generate_from_waypoints(self, waypoints: List[Waypoint]) -> None:
        """
        Generate RAPID code from optimized waypoints.

        Uses MoveL for linear segments, MoveAbsJ otherwise.
        """
        self.targets = []
        self.instructions = []
        self.target_counter = 0

        prev_segment_id = -1
        in_linear_segment = False

        for wp in waypoints:
            if wp.move_type == MoveType.MOVEL and wp.is_linear:
                # Use MoveL with robtarget
                target_name = self.add_rob_target(wp)

                # Add comment when starting new linear segment
                if wp.segment_id != prev_segment_id:
                    self.instructions.append(f"        ! Linear segment {wp.segment_id + 1}")
                    prev_segment_id = wp.segment_id

                self.add_movel(target_name)
                in_linear_segment = True
            else:
                # Use MoveAbsJ with jointtarget
                target_name = self.add_joint_target(wp)

                if in_linear_segment:
                    self.instructions.append("        ! Joint movement")
                    in_linear_segment = False

                self.add_movej(target_name)

    def generate_mod_file(self) -> str:
        """Generate complete RAPID .mod file content."""
        lines = [
            f"MODULE {self.module_name}",
            "",
            "    ! ================================================",
            "    ! Auto-generated RAPID code from MoveIt trajectory",
            "    ! ================================================",
            "",
            "    ! Speed configuration",
            f"    CONST speeddata motion_speed := [{self.speed_tcp},{self.speed_ori},{self.speed_ori},{self.speed_ori}];",
            "",
            "    ! Target positions",
        ]

        # Add targets
        lines.extend(self.targets)

        lines.extend([
            "",
            "    PROC main()",
            "        ! Initialize",
            "        ConfJ \\Off;",
            "        ConfL \\Off;",
            "",
            "        ! Execute trajectory",
        ])

        # Add instructions
        lines.extend(self.instructions)

        lines.extend([
            "",
            "        ! Done",
            "        Stop;",
            "    ENDPROC",
            "",
            "ENDMODULE",
        ])

        return "\n".join(lines)


class TrajectoryToRAPID:
    """
    Main converter class for MoveIt trajectory to RAPID code.

    Usage:
        converter = TrajectoryToRAPID()
        converter.load_trajectory(trajectory_msg)
        converter.detect_linear_segments(tolerance=0.001)
        rapid_code = converter.export_rapid("output.mod")
    """

    # Joint names matching the URDF
    JOINT_NAMES = [
        'irb120_joint_1',
        'irb120_joint_2',
        'irb120_joint_3',
        'irb120_joint_4',
        'irb120_joint_5',
        'irb120_joint_6',
    ]

    def __init__(self):
        self.waypoints: List[Waypoint] = []
        self.segments: List[LinearSegment] = []
        self.fk = IRB120ForwardKinematics()
        self.detector = None
        self.generator = None

    def load_trajectory(self, trajectory) -> int:
        """
        Load a JointTrajectory message.

        Args:
            trajectory: ROS JointTrajectory message

        Returns:
            Number of waypoints loaded
        """
        self.waypoints = []
        self.segments = []

        # Map joint names to indices
        joint_indices = []
        for name in self.JOINT_NAMES:
            if name in trajectory.joint_names:
                joint_indices.append(trajectory.joint_names.index(name))
            else:
                raise ValueError(f"Joint {name} not found in trajectory")

        # Extract waypoints
        for i, point in enumerate(trajectory.points):
            joints = [point.positions[idx] for idx in joint_indices]

            # Calculate time from start
            time_from_start = (
                point.time_from_start.sec +
                point.time_from_start.nanosec * 1e-9
            )

            wp = Waypoint(
                joint_positions=joints,
                time_from_start=time_from_start,
                index=i
            )
            self.waypoints.append(wp)

        return len(self.waypoints)

    def load_from_joint_list(self, joint_list: List[List[float]]) -> int:
        """
        Load trajectory from list of joint positions (for testing).

        Args:
            joint_list: List of joint position lists (radians)

        Returns:
            Number of waypoints loaded
        """
        self.waypoints = []
        self.segments = []

        for i, joints in enumerate(joint_list):
            if len(joints) != 6:
                raise ValueError(f"Expected 6 joints, got {len(joints)}")

            wp = Waypoint(
                joint_positions=list(joints),
                time_from_start=float(i),
                index=i
            )
            self.waypoints.append(wp)

        return len(self.waypoints)

    def compute_cartesian_poses(self) -> None:
        """Compute Cartesian poses for all waypoints using FK."""
        for wp in self.waypoints:
            wp.cartesian_pose = self.fk.compute_fk(wp.joint_positions)

    def detect_linear_segments(self, tolerance: float = 0.001) -> Tuple[int, int]:
        """
        Detect linear segments for MoveL optimization.

        Args:
            tolerance: Maximum deviation from line in meters (default 1mm)

        Returns:
            Tuple of (num_linear_points, num_total_points)
        """
        self.detector = LinearSegmentDetector(tolerance)

        # Compute FK first
        self.detector.compute_cartesian_poses(self.waypoints)

        # Detect segments
        self.segments = self.detector.detect_segments(self.waypoints)

        # Count linear points
        linear_count = sum(1 for wp in self.waypoints if wp.is_linear)

        return linear_count, len(self.waypoints)

    def export_rapid(self, filename: str = None,
                     module_name: str = "GeneratedProgram",
                     speed: int = 100,
                     zone: str = "z1") -> str:
        """
        Export trajectory to RAPID .mod file.

        Args:
            filename: Output file path (optional)
            module_name: RAPID module name
            speed: TCP speed in mm/s
            zone: Zone data

        Returns:
            Generated RAPID code as string
        """
        self.generator = RAPIDGenerator(
            module_name=module_name,
            speed_tcp=speed,
            zone=zone
        )

        # Generate from waypoints
        self.generator.generate_from_waypoints(self.waypoints)

        # Get code
        rapid_code = self.generator.generate_mod_file()

        # Write to file if specified
        if filename:
            with open(filename, 'w') as f:
                f.write(rapid_code)

        return rapid_code

    def get_statistics(self) -> dict:
        """Get conversion statistics."""
        total_waypoints = len(self.waypoints)
        linear_waypoints = sum(1 for wp in self.waypoints if wp.is_linear)
        num_segments = len(self.segments)
        significant_segments = sum(1 for s in self.segments if s.is_significant)

        # Count actual RAPID commands
        movej_count = sum(1 for wp in self.waypoints if wp.move_type == MoveType.MOVEJ)
        movel_count = sum(1 for wp in self.waypoints if wp.move_type == MoveType.MOVEL)

        return {
            'total_waypoints': total_waypoints,
            'linear_waypoints': linear_waypoints,
            'non_linear_waypoints': total_waypoints - linear_waypoints,
            'num_segments': num_segments,
            'significant_linear_segments': significant_segments,
            'movej_commands': movej_count,
            'movel_commands': movel_count,
            'total_commands': movej_count + movel_count,
            'optimization_ratio': (total_waypoints - (movej_count + movel_count)) / total_waypoints if total_waypoints > 0 else 0
        }


class OptimizedSocketExecutor:
    """
    Execute optimized trajectories via socket streaming.

    Uses linear segment detection to reduce waypoints,
    sending only key waypoints to the robot.
    """

    def __init__(self, robot_ip: str = '192.168.125.1', robot_port: int = 5000):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.socket = None
        self.fk = IRB120ForwardKinematics()

    def connect(self) -> bool:
        """Connect to robot socket server."""
        import socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(10)
        try:
            self.socket.connect((self.robot_ip, self.robot_port))
            # Test connection
            self.socket.send(b'PING\n')
            import time
            time.sleep(0.5)
            response = self.socket.recv(1024).decode().strip()
            return response == 'PONG'
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from robot."""
        if self.socket:
            try:
                self.socket.send(b'QUIT\n')
                self.socket.close()
            except:
                pass
            self.socket = None

    def get_position(self) -> List[float]:
        """Get current joint positions in radians."""
        import time
        self.socket.send(b'GETPOS\n')
        time.sleep(0.3)
        response = self.socket.recv(1024).decode().strip()
        if response.startswith('POS:'):
            joints_deg = [float(x) for x in response[4:].split()]
            return [math.radians(j) for j in joints_deg]
        return None

    def move_joints(self, joints_rad: List[float], timeout: float = 60.0) -> bool:
        """Move to joint position (radians)."""
        import time
        joints_deg = [math.degrees(j) for j in joints_rad]
        cmd = 'MOVEJ ' + ' '.join([f'{j:.2f}' for j in joints_deg]) + '\n'

        self.socket.settimeout(timeout)
        self.socket.send(cmd.encode())
        time.sleep(0.1)

        try:
            response = self.socket.recv(1024).decode().strip()
            return 'OK' in response
        except:
            return False

    def set_speed(self, speed_mm_s: int) -> bool:
        """Set robot speed."""
        import time
        self.socket.send(f'SPEED {speed_mm_s}\n'.encode())
        time.sleep(0.3)
        response = self.socket.recv(1024).decode().strip()
        return 'OK' in response

    def execute_optimized(self, trajectory, tolerance: float = 0.001,
                          speed: int = 50, verbose: bool = True) -> bool:
        """
        Execute trajectory with linear optimization.

        Only sends key waypoints (start/end of linear segments),
        reducing the number of commands significantly.

        Args:
            trajectory: JointTrajectory message
            tolerance: Linear detection tolerance in meters (default 1mm)
            speed: Robot speed in mm/s
            verbose: Print progress

        Returns:
            True if execution successful
        """
        # Convert trajectory
        converter = TrajectoryToRAPID()
        converter.load_trajectory(trajectory)

        # Detect linear segments
        converter.detect_linear_segments(tolerance=tolerance)

        # Get optimized waypoints (only segment endpoints)
        optimized_waypoints = self._get_optimized_waypoints(converter)

        if verbose:
            original_count = len(converter.waypoints)
            optimized_count = len(optimized_waypoints)
            reduction = (1 - optimized_count / original_count) * 100
            print(f"Optimization: {original_count} -> {optimized_count} waypoints ({reduction:.0f}% reduction)")

        # Set speed
        self.set_speed(speed)

        # Execute each optimized waypoint
        for i, wp in enumerate(optimized_waypoints):
            if verbose:
                pose = wp.cartesian_pose
                print(f"  [{i+1}/{len(optimized_waypoints)}] Moving to ({pose.x*1000:.1f}, {pose.y*1000:.1f}, {pose.z*1000:.1f}) mm")

            if not self.move_joints(wp.joint_positions):
                print(f"  ERROR: Failed at waypoint {i+1}")
                return False

        if verbose:
            print("  Done!")

        return True

    def _get_optimized_waypoints(self, converter: 'TrajectoryToRAPID') -> List[Waypoint]:
        """
        Extract only key waypoints from trajectory.

        For linear segments: keep start and end points only
        For non-linear: keep all points
        """
        if not converter.segments:
            return converter.waypoints

        optimized = []
        seen_indices = set()

        for segment in converter.segments:
            if segment.is_significant:
                # Linear segment: only keep start and end
                if segment.start_idx not in seen_indices:
                    optimized.append(converter.waypoints[segment.start_idx])
                    seen_indices.add(segment.start_idx)
                if segment.end_idx not in seen_indices:
                    optimized.append(converter.waypoints[segment.end_idx])
                    seen_indices.add(segment.end_idx)
            else:
                # Non-linear: keep all points in segment
                for idx in range(segment.start_idx, segment.end_idx + 1):
                    if idx not in seen_indices:
                        optimized.append(converter.waypoints[idx])
                        seen_indices.add(idx)

        return optimized

    def execute_cartesian_move(self, dx: float = 0, dy: float = 0, dz: float = 0,
                               num_waypoints: int = 10, tolerance: float = 0.001,
                               speed: int = 50, verbose: bool = True) -> bool:
        """
        Execute a Cartesian movement relative to current position.

        Args:
            dx, dy, dz: Relative movement in meters
            num_waypoints: Number of interpolation points
            tolerance: Linear detection tolerance
            speed: Robot speed in mm/s
            verbose: Print progress

        Returns:
            True if successful
        """
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        # Get current position
        current_joints = self.get_position()
        if not current_joints:
            print("ERROR: Could not get current position")
            return False

        # Compute current TCP
        current_pose = self.fk.compute_fk(current_joints)

        if verbose:
            print(f"Current TCP: ({current_pose.x*1000:.1f}, {current_pose.y*1000:.1f}, {current_pose.z*1000:.1f}) mm")
            print(f"Movement: dx={dx*1000:.1f}mm, dy={dy*1000:.1f}mm, dz={dz*1000:.1f}mm")

        # Target position
        target_x = current_pose.x + dx
        target_y = current_pose.y + dy
        target_z = current_pose.z + dz

        if verbose:
            print(f"Target TCP: ({target_x*1000:.1f}, {target_y*1000:.1f}, {target_z*1000:.1f}) mm")

        # Create trajectory using IK
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'irb120_joint_1', 'irb120_joint_2', 'irb120_joint_3',
            'irb120_joint_4', 'irb120_joint_5', 'irb120_joint_6',
        ]

        # Simple numerical IK
        prev_joints = current_joints

        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)

            # Interpolate
            interp_x = current_pose.x + t * dx
            interp_y = current_pose.y + t * dy
            interp_z = current_pose.z + t * dz

            # Solve IK
            target_pos = np.array([interp_x, interp_y, interp_z])
            joints = self._solve_ik(target_pos, prev_joints)

            point = JointTrajectoryPoint()
            point.positions = joints
            point.velocities = [0.0] * 6
            point.time_from_start = Duration(sec=i, nanosec=0)
            trajectory.points.append(point)

            prev_joints = joints

        # Execute with optimization
        return self.execute_optimized(trajectory, tolerance=tolerance,
                                      speed=speed, verbose=verbose)

    def _solve_ik(self, target_pos: np.ndarray, initial_joints: List[float],
                  max_iter: int = 100, tol: float = 0.0005) -> List[float]:
        """Simple numerical IK using Jacobian transpose."""
        joints = np.array(initial_joints)

        for _ in range(max_iter):
            pose = self.fk.compute_fk(joints.tolist())
            current_pos = np.array([pose.x, pose.y, pose.z])
            error = target_pos - current_pos

            if np.linalg.norm(error) < tol:
                break

            # Compute Jacobian numerically
            J = np.zeros((3, 6))
            delta = 0.0001
            for i in range(6):
                joints_plus = joints.copy()
                joints_plus[i] += delta
                pose_plus = self.fk.compute_fk(joints_plus.tolist())
                pos_plus = np.array([pose_plus.x, pose_plus.y, pose_plus.z])
                J[:, i] = (pos_plus - current_pos) / delta

            # Jacobian transpose step
            delta_q = 0.5 * J.T @ error
            joints += delta_q

            # Limit to joint limits
            limits = [(-2.87, 2.87), (-1.92, 1.92), (-1.92, 1.22),
                      (-2.79, 2.79), (-2.0, 2.0), (-6.98, 6.98)]
            for i in range(6):
                joints[i] = np.clip(joints[i], limits[i][0], limits[i][1])

        return joints.tolist()


# Command-line testing
if __name__ == "__main__":
    import sys

    print("=" * 60)
    print("TrajectoryToRAPID - Test Suite")
    print("=" * 60)

    # Test FK
    print("\n1. Testing Forward Kinematics")
    print("-" * 40)

    fk = IRB120ForwardKinematics()

    # Test at home position (all zeros)
    home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    home_pose = fk.compute_fk(home_joints)
    print(f"Home position FK:")
    print(f"  Joints: {[f'{j:.2f}' for j in home_joints]} rad")
    print(f"  Pose: x={home_pose.x:.4f}, y={home_pose.y:.4f}, z={home_pose.z:.4f} m")
    print(f"  Quaternion: [{home_pose.qw:.4f}, {home_pose.qx:.4f}, {home_pose.qy:.4f}, {home_pose.qz:.4f}]")

    # Expected: X should be ~L4+L5 = 0.374m, Z should be ~L1+L2+L3 = 0.63m
    expected_x = 0.374  # L4 + L5
    expected_z = 0.63   # L1 + L2 + L3

    print(f"\n  Expected X ~{expected_x:.3f}m (L4+L5), got {home_pose.x:.4f}m")
    print(f"  Expected Z ~{expected_z:.3f}m (L1+L2+L3), got {home_pose.z:.4f}m")

    # Test with J1 = 90 degrees
    print("\n2. Testing FK with J1 = 90°")
    print("-" * 40)

    j1_90 = [math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0]
    pose_j1_90 = fk.compute_fk(j1_90)
    print(f"  Joints: J1=90°, others=0°")
    print(f"  Pose: x={pose_j1_90.x:.4f}, y={pose_j1_90.y:.4f}, z={pose_j1_90.z:.4f} m")
    # With J1=90°, X should swap with Y
    print(f"  Expected: Y ~{expected_x:.3f}m, X ~0, Z ~{expected_z:.3f}m")

    # Test linear segment detection
    print("\n3. Testing Linear Segment Detection")
    print("-" * 40)

    # Create a simple linear trajectory (straight line along X)
    converter = TrajectoryToRAPID()

    # Generate points along a line (varying J2 slightly creates linear motion)
    test_trajectory = []
    for i in range(10):
        # Small variations in J2 create near-linear TCP motion
        joints = [0.0, math.radians(i * 2), 0.0, 0.0, 0.0, 0.0]
        test_trajectory.append(joints)

    converter.load_from_joint_list(test_trajectory)
    linear_count, total_count = converter.detect_linear_segments(tolerance=0.01)  # 10mm tolerance for test

    print(f"  Test trajectory: {total_count} waypoints")
    print(f"  Linear waypoints detected: {linear_count}")

    stats = converter.get_statistics()
    print(f"  MoveJ commands: {stats['movej_commands']}")
    print(f"  MoveL commands: {stats['movel_commands']}")

    # Generate RAPID code
    print("\n4. Testing RAPID Code Generation")
    print("-" * 40)

    rapid_code = converter.export_rapid(
        module_name="TestProgram",
        speed=100,
        zone="z1"
    )

    # Show first and last parts
    lines = rapid_code.split('\n')
    print("  Generated RAPID code (first 15 lines):")
    for line in lines[:15]:
        print(f"    {line}")
    print("    ...")
    print(f"  Total lines: {len(lines)}")

    print("\n" + "=" * 60)
    print("Tests completed!")
    print("=" * 60)
