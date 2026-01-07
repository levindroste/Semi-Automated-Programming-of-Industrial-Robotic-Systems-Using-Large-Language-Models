#!/usr/bin/env python3
"""
Node to add collision objects to MoveIt planning scene.

Adds objects from AML configuration file or hardcoded defaults.
Uses MoveIt2 ApplyPlanningScene service for reliable object management.

Usage:
    ros2 run ur10e_hl_interface add_collision_objects.py

Parameters:
    - mesh_directory: Path to STL mesh files
    - aml_file: Path to AML configuration file (optional)
    - use_sim_time: Whether to use simulation time
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from std_msgs.msg import ColorRGBA
import struct
import os
import time
import xml.etree.ElementTree as ET


class AddCollisionObjectsNode(Node):
    """Node that adds collision objects to the MoveIt planning scene."""

    # Color definitions (RGB, 0-1 range)
    COLORS = {
        'Grau': (0.5, 0.5, 0.5),
        'Schwarz': (0.2, 0.2, 0.2),
        'Rot': (1.0, 0.0, 0.0),
        'Gruen': (0.0, 1.0, 0.0),
        'Gelb': (1.0, 1.0, 0.0),
        'Blau': (0.0, 0.0, 1.0),
        'Orange': (1.0, 0.5, 0.0),
    }

    def __init__(self):
        super().__init__('add_collision_objects')

        # Declare parameters
        self.declare_parameter('mesh_directory', '')
        self.declare_parameter('aml_file', '')
        # Note: use_sim_time is automatically declared by ROS, don't redeclare

        self.mesh_directory = self.get_parameter('mesh_directory').get_parameter_value().string_value
        self.aml_file = self.get_parameter('aml_file').get_parameter_value().string_value

        self.get_logger().info(f'Mesh directory: {self.mesh_directory}')
        self.get_logger().info(f'AML file: {self.aml_file}')

        # Callback group for service calls
        self.callback_group = ReentrantCallbackGroup()

        # Service client for applying planning scene (more reliable than topic)
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
            callback_group=self.callback_group
        )

        # Wait for MoveIt to be ready
        self.get_logger().info('Waiting for /apply_planning_scene service...')
        self.create_timer(2.0, self.add_objects_callback, callback_group=self.callback_group)

        # Timer to verify objects were added
        self.verify_timer = None
        self.objects_added = False

    def add_objects_callback(self):
        """Add all collision objects to the planning scene."""
        if self.objects_added:
            return

        # Wait for service to be available
        if not self.apply_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/apply_planning_scene service not available!')
            return

        self.get_logger().info('Adding collision objects to planning scene via service...')

        # Define objects to add (hardcoded for reliability)
        objects = self.get_objects_to_add()

        if not objects:
            self.get_logger().error('No objects to add!')
            return

        # Create planning scene message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        collision_objects = []
        object_colors = []

        for obj_def in objects:
            self.get_logger().info(f"Creating object: {obj_def['name']}")

            if obj_def['type'] == 'box':
                co = self.create_box_object(obj_def)
            elif obj_def['type'] == 'mesh':
                co = self.create_mesh_object(obj_def)
            else:
                self.get_logger().warn(f"Unknown object type: {obj_def['type']}")
                continue

            if co is not None:
                collision_objects.append(co)
                object_colors.append(self.create_object_color(
                    obj_def['name'],
                    obj_def.get('color', 'Grau')
                ))
                self.get_logger().info(f"Successfully created: {obj_def['name']}")
            else:
                self.get_logger().error(f"Failed to create: {obj_def['name']}")

        scene_msg.world.collision_objects = collision_objects
        scene_msg.object_colors = object_colors

        # Use service call for reliable delivery
        self.get_logger().info(f'Applying {len(collision_objects)} collision objects via service...')

        request = ApplyPlanningScene.Request()
        request.scene = scene_msg

        future = self.apply_scene_client.call_async(request)
        future.add_done_callback(self.apply_scene_callback)

    def apply_scene_callback(self, future):
        """Handle the response from apply_planning_scene service."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Collision objects applied successfully!')
                self.objects_added = True
            else:
                self.get_logger().error('Failed to apply collision objects!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def get_objects_to_add(self):
        """Return list of objects to add from AML file."""
        objects = []

        # Always add the mesh_object (Hutschienen-Halter)
        objects.append({
            'name': 'mesh_object',
            'type': 'mesh',
            'mesh_file': 'Mesh.stl',
            'position': [0.2971, -0.0962, 0.0],
            'orientation': [0.0, 0.0, 0.0, 1.0],
            'scale': 0.001,
            'color': 'Grau',
        })

        # Try to load objects from AML file
        aml_objects = self.parse_aml_objects()
        if aml_objects:
            objects.extend(aml_objects)
            self.get_logger().info(f'Loaded {len(aml_objects)} objects from AML file')
        else:
            self.get_logger().warn('No objects found in AML file or AML parsing failed')

        return objects

    def parse_aml_objects(self):
        """Parse AML file and return list of objects to spawn."""
        # Find AML file
        aml_path = self.aml_file
        if not aml_path:
            # Default path
            aml_path = os.path.expanduser(
                '~/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models'
                '/src/ur10e_hl_interface/config/irb120_simple_config.aml'
            )

        if not os.path.exists(aml_path):
            self.get_logger().error(f'AML file not found: {aml_path}')
            return []

        self.get_logger().info(f'Parsing AML file: {aml_path}')

        try:
            tree = ET.parse(aml_path)
            root = tree.getroot()

            # Handle XML namespace
            ns = {'aml': 'http://www.dke.de/CAEX'}

            # First, parse grid positions to get XYZ coordinates
            grid_positions = self.parse_grid_positions(root, ns)
            if not grid_positions:
                self.get_logger().warn('No grid positions found in AML')
                return []

            self.get_logger().info(f'Parsed {len(grid_positions)} grid positions')

            # Then, parse objects and map their locations to positions
            objects = self.parse_objects_from_aml(root, ns, grid_positions)
            return objects

        except ET.ParseError as e:
            self.get_logger().error(f'Error parsing AML file: {e}')
            return []
        except Exception as e:
            self.get_logger().error(f'Unexpected error parsing AML: {e}')
            return []

    def parse_grid_positions(self, root, ns):
        """Parse Grid-Config hierarchy and calculate grid positions dynamically."""
        grid_positions = {}

        # Default values
        a1_spawn = [0.495, -0.090, 0.0]
        grid_spacing = 0.0475

        # Find Grid-Config InstanceHierarchy
        for hierarchy in root.findall('.//aml:InstanceHierarchy', ns):
            if hierarchy.get('Name') == 'Grid-Config':
                for elem in hierarchy.findall('aml:InternalElement', ns):
                    name = elem.get('Name')
                    if not name:
                        continue

                    if name == 'GridParameters':
                        # Parse A1SpawnPosition, GridSpacing
                        for attr in elem.findall('aml:Attribute', ns):
                            attr_name = attr.get('Name')
                            value_elem = attr.find('aml:Value', ns)
                            if value_elem is None or not value_elem.text:
                                continue

                            if attr_name == 'A1SpawnPosition':
                                try:
                                    parts = value_elem.text.split(',')
                                    if len(parts) >= 3:
                                        a1_spawn = [float(p) for p in parts[:3]]
                                except ValueError:
                                    pass
                            elif attr_name == 'GridSpacing':
                                try:
                                    grid_spacing = float(value_elem.text)
                                except ValueError:
                                    pass

                    elif name == 'SpecialPositions':
                        # Parse special positions like "X"
                        for special in elem.findall('aml:InternalElement', ns):
                            special_name = special.get('Name')
                            if not special_name:
                                continue

                            for attr in special.findall('aml:Attribute', ns):
                                if attr.get('Name') == 'Position':
                                    value_elem = attr.find('aml:Value', ns)
                                    if value_elem is not None and value_elem.text:
                                        try:
                                            parts = value_elem.text.split(',')
                                            if len(parts) >= 3:
                                                grid_positions[special_name] = [float(p) for p in parts[:3]]
                                        except ValueError:
                                            pass

        # Generate all grid positions A1-E5 from parameters
        for col_idx, col in enumerate('ABCDE'):
            for row in range(1, 6):
                grid_name = f"{col}{row}"
                x = a1_spawn[0] - ((row - 1) * grid_spacing)
                y = a1_spawn[1] + (col_idx * grid_spacing)
                z = a1_spawn[2]
                grid_positions[grid_name] = [x, y, z]

        self.get_logger().info(f'Grid config: A1={a1_spawn}, spacing={grid_spacing}m')
        return grid_positions

    def parse_objects_from_aml(self, root, ns, grid_positions):
        """Parse Objects hierarchy and return list of object definitions."""
        objects = []

        # Find Objects InstanceHierarchy
        for hierarchy in root.findall('.//aml:InstanceHierarchy', ns):
            if hierarchy.get('Name') == 'Objects':
                for elem in hierarchy.findall('aml:InternalElement', ns):
                    name = elem.get('Name')
                    if not name:
                        continue

                    # Get attributes
                    location = None
                    color = 'Grau'
                    dimensions = [0.03, 0.03, 0.03]  # Default cube size

                    for attr in elem.findall('aml:Attribute', ns):
                        attr_name = attr.get('Name')
                        value_elem = attr.find('aml:Value', ns)
                        if value_elem is None or not value_elem.text:
                            continue

                        if attr_name == 'Location':
                            location = value_elem.text
                        elif attr_name == 'Color':
                            color = value_elem.text
                        elif attr_name == 'Dimensions':
                            try:
                                parts = value_elem.text.split(',')
                                if len(parts) >= 3:
                                    dimensions = [float(p) for p in parts[:3]]
                            except ValueError:
                                pass

                    # Skip objects without valid location
                    if not location:
                        self.get_logger().warn(f'Object {name} has no location, skipping')
                        continue

                    # Handle stacked positions (e.g., "A1:1" -> base="A1", level=1)
                    base_location = location
                    stack_level = 0
                    if ':' in location:
                        parts = location.split(':')
                        base_location = parts[0]
                        try:
                            stack_level = int(parts[1])
                        except ValueError:
                            pass

                    # Look up position from grid using base location
                    if base_location not in grid_positions:
                        self.get_logger().warn(f'Location {base_location} not found in grid for object {name}')
                        continue

                    pos = grid_positions[base_location]
                    # Spawn at base Z + stack level * cube height (0.03m per level)
                    cube_height = 0.03
                    spawn_z = pos[2] + (stack_level * cube_height)
                    spawn_pos = [pos[0], pos[1], spawn_z]

                    self.get_logger().info(f'Object {name}: location={location}, pos={spawn_pos}, color={color}')

                    objects.append({
                        'name': name,
                        'type': 'mesh',
                        'mesh_file': 'Wuerfel.stl',
                        'position': spawn_pos,
                        'orientation': [0.0, 0.0, 0.0, 1.0],
                        'scale': 0.001,
                        'color': color,
                    })

        return objects

    def create_box_object(self, obj_def):
        """Create a box collision object."""
        co = CollisionObject()
        co.header.frame_id = 'world'
        co.id = obj_def['name']
        co.operation = CollisionObject.ADD

        # Create box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = obj_def['dimensions']

        # Create pose
        pose = Pose()
        pose.position.x = obj_def['position'][0]
        pose.position.y = obj_def['position'][1]
        pose.position.z = obj_def['position'][2]
        pose.orientation.x = obj_def['orientation'][0]
        pose.orientation.y = obj_def['orientation'][1]
        pose.orientation.z = obj_def['orientation'][2]
        pose.orientation.w = obj_def['orientation'][3]

        co.primitives.append(box)
        co.primitive_poses.append(pose)

        return co

    def create_mesh_object(self, obj_def):
        """Create a mesh collision object from STL file."""
        mesh_path = os.path.join(self.mesh_directory, obj_def['mesh_file'])

        if not os.path.exists(mesh_path):
            self.get_logger().error(f"Mesh file not found: {mesh_path}")
            return None

        self.get_logger().info(f"Loading mesh: {mesh_path}")

        # Load STL file
        mesh = self.load_stl_mesh(mesh_path, obj_def.get('scale', 1.0))
        if mesh is None:
            return None

        co = CollisionObject()
        co.header.frame_id = 'world'
        co.id = obj_def['name']
        co.operation = CollisionObject.ADD

        # Create pose
        pose = Pose()
        pose.position.x = obj_def['position'][0]
        pose.position.y = obj_def['position'][1]
        pose.position.z = obj_def['position'][2]
        pose.orientation.x = obj_def['orientation'][0]
        pose.orientation.y = obj_def['orientation'][1]
        pose.orientation.z = obj_def['orientation'][2]
        pose.orientation.w = obj_def['orientation'][3]

        co.meshes.append(mesh)
        co.mesh_poses.append(pose)

        return co

    def load_stl_mesh(self, filepath, scale=1.0):
        """Load an ASCII STL file and return a Mesh message."""
        try:
            vertices = []
            triangles = []

            with open(filepath, 'r') as f:
                content = f.read()

            # Check if it's ASCII STL
            if not content.strip().startswith('solid'):
                self.get_logger().error(f"Not an ASCII STL file: {filepath}")
                return self.load_binary_stl(filepath, scale)

            # Parse ASCII STL
            lines = content.split('\n')
            current_triangle_vertices = []

            for line in lines:
                line = line.strip()
                if line.startswith('vertex'):
                    parts = line.split()
                    if len(parts) >= 4:
                        x = float(parts[1]) * scale
                        y = float(parts[2]) * scale
                        z = float(parts[3]) * scale

                        # Find or add vertex
                        vertex_idx = len(vertices)
                        point = Point()
                        point.x = x
                        point.y = y
                        point.z = z
                        vertices.append(point)
                        current_triangle_vertices.append(vertex_idx)

                elif line.startswith('endfacet'):
                    if len(current_triangle_vertices) == 3:
                        triangle = MeshTriangle()
                        triangle.vertex_indices = current_triangle_vertices
                        triangles.append(triangle)
                    current_triangle_vertices = []

            mesh = Mesh()
            mesh.vertices = vertices
            mesh.triangles = triangles

            self.get_logger().info(f"Loaded mesh: {len(vertices)} vertices, {len(triangles)} triangles")
            return mesh

        except Exception as e:
            self.get_logger().error(f"Error loading STL: {e}")
            return None

    def load_binary_stl(self, filepath, scale=1.0):
        """Load a binary STL file and return a Mesh message."""
        try:
            with open(filepath, 'rb') as f:
                # Skip header (80 bytes)
                f.read(80)

                # Read number of triangles
                num_triangles = struct.unpack('<I', f.read(4))[0]

                vertices = []
                triangles = []

                for _ in range(num_triangles):
                    # Skip normal (12 bytes)
                    f.read(12)

                    triangle_vertices = []
                    for _ in range(3):
                        x, y, z = struct.unpack('<fff', f.read(12))
                        vertex_idx = len(vertices)
                        point = Point()
                        point.x = x * scale
                        point.y = y * scale
                        point.z = z * scale
                        vertices.append(point)
                        triangle_vertices.append(vertex_idx)

                    triangle = MeshTriangle()
                    triangle.vertex_indices = triangle_vertices
                    triangles.append(triangle)

                    # Skip attribute (2 bytes)
                    f.read(2)

            mesh = Mesh()
            mesh.vertices = vertices
            mesh.triangles = triangles

            self.get_logger().info(f"Loaded binary mesh: {len(vertices)} vertices, {len(triangles)} triangles")
            return mesh

        except Exception as e:
            self.get_logger().error(f"Error loading binary STL: {e}")
            return None

    def create_object_color(self, object_id, color_name):
        """Create an ObjectColor message."""
        oc = ObjectColor()
        oc.id = object_id

        rgb = self.COLORS.get(color_name, (0.5, 0.5, 0.5))
        oc.color.r = rgb[0]
        oc.color.g = rgb[1]
        oc.color.b = rgb[2]
        oc.color.a = 1.0

        return oc


def main(args=None):
    rclpy.init(args=args)

    node = AddCollisionObjectsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
