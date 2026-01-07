# irb120_aml_parser.py
# Parser for IRB 120 configuration from irb120_simple_config.aml
# Reads grid positions and object states for dynamic prompt generation

import xml.etree.ElementTree as ET
import os
from typing import Dict, List, Tuple, Optional, Any


class IRB120AMLParser:
    """Singleton Parser for IRB 120 AML configuration"""

    _instance = None
    _initialized = False

    def __new__(cls):
        """Singleton Pattern - only one instance"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """Initialize parser once"""
        if self._initialized:
            return

        # Path to AML config
        base = os.path.expanduser(
            "~/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models"
            "/src/ur10e_hl_interface/config"
        )
        self.config_path = f"{base}/irb120_simple_config.aml"

        # Data storage
        self.grid_positions = {}  # Grid name -> [x, y, z] (spawn positions)
        self.objects = {}         # Object name -> {location, color, dimensions}

        # Grid configuration (for dynamic position calculation)
        self.a1_spawn_position = [0.495, -0.090, 0.0]  # Default A1 spawn
        self.grid_spacing = 0.0475  # 47.5mm default
        self.grip_offset = [0.015, 0.015, 0.1]  # Default grip offset

        # XML namespace
        self.ns = {'aml': 'http://www.dke.de/CAEX'}

        # Cache for formatted outputs
        self._cache = {}

        # Auto-parse on initialization
        self.reload()
        self._initialized = True

    def reload(self):
        """Reload configuration from AML file"""
        self._cache.clear()
        self.grid_positions.clear()
        self.objects.clear()
        self.parse_config()

    def parse_config(self) -> bool:
        """Parse the AML configuration file"""
        if not os.path.exists(self.config_path):
            print(f"AML config not found: {self.config_path}")
            return False

        try:
            tree = ET.parse(self.config_path)
            root = tree.getroot()

            # Parse hierarchies
            for hierarchy in root.findall('.//aml:InstanceHierarchy', self.ns):
                name = hierarchy.get('Name', '')
                if name == 'Grid-Config':
                    self._parse_grid_config(hierarchy)
                elif name == 'Objects':
                    self._parse_objects(hierarchy)

            return True

        except ET.ParseError as e:
            print(f"XML parse error: {e}")
            return False
        except Exception as e:
            print(f"Config parse error: {e}")
            return False

    def _parse_grid_config(self, hierarchy: ET.Element):
        """Parse Grid-Config hierarchy and generate grid positions"""
        for elem in hierarchy.findall('aml:InternalElement', self.ns):
            name = elem.get('Name')
            if not name:
                continue

            if name == 'GridParameters':
                # Parse A1SpawnPosition
                for attr in elem.findall('aml:Attribute', self.ns):
                    attr_name = attr.get('Name')
                    value_elem = attr.find('aml:Value', self.ns)
                    if value_elem is None or not value_elem.text:
                        continue

                    if attr_name == 'A1SpawnPosition':
                        try:
                            parts = value_elem.text.split(',')
                            if len(parts) >= 3:
                                self.a1_spawn_position = [float(p) for p in parts[:3]]
                        except ValueError:
                            pass
                    elif attr_name == 'GridSpacing':
                        try:
                            self.grid_spacing = float(value_elem.text)
                        except ValueError:
                            pass
                    elif attr_name == 'GripOffset':
                        try:
                            parts = value_elem.text.split(',')
                            if len(parts) >= 3:
                                self.grip_offset = [float(p) for p in parts[:3]]
                        except ValueError:
                            pass

            elif name == 'SpecialPositions':
                # Parse special positions like "X"
                for special in elem.findall('aml:InternalElement', self.ns):
                    special_name = special.get('Name')
                    if not special_name:
                        continue

                    for attr in special.findall('aml:Attribute', self.ns):
                        if attr.get('Name') == 'Position':
                            value_elem = attr.find('aml:Value', self.ns)
                            if value_elem is not None and value_elem.text:
                                try:
                                    parts = value_elem.text.split(',')
                                    if len(parts) >= 3:
                                        self.grid_positions[special_name] = [float(p) for p in parts[:3]]
                                except ValueError:
                                    pass

        # Generate all grid positions A1-E5 from parameters
        for col_idx, col in enumerate('ABCDE'):
            for row in range(1, 6):
                grid_name = f"{col}{row}"
                self.grid_positions[grid_name] = self._calculate_spawn_position(grid_name)

    def _parse_objects(self, hierarchy: ET.Element):
        """Parse Objects hierarchy"""
        for elem in hierarchy.findall('aml:InternalElement', self.ns):
            name = elem.get('Name')
            if not name:
                continue

            obj_data = {
                'name': name,
                'location': None,
                'color': 'Grau',
                'dimensions': [0.03, 0.03, 0.03]
            }

            for attr in elem.findall('aml:Attribute', self.ns):
                attr_name = attr.get('Name')
                value_elem = attr.find('aml:Value', self.ns)
                if value_elem is None or not value_elem.text:
                    continue

                if attr_name == 'Location':
                    obj_data['location'] = value_elem.text
                elif attr_name == 'Color':
                    obj_data['color'] = value_elem.text
                elif attr_name == 'Dimensions':
                    try:
                        parts = value_elem.text.split(',')
                        if len(parts) >= 3:
                            obj_data['dimensions'] = [float(p) for p in parts[:3]]
                    except ValueError:
                        pass

            self.objects[name] = obj_data

    # ========== POSITION CALCULATION ==========

    def _calculate_spawn_position(self, grid_name: str) -> List[float]:
        """Calculate spawn position for a grid name (e.g., 'A1', 'C3')"""
        # Check for special positions first
        if grid_name in self.grid_positions and len(grid_name) == 1:
            return self.grid_positions[grid_name]

        # Parse grid name (e.g., "A1" -> col='A', row=1)
        if len(grid_name) >= 2 and grid_name[0].isalpha() and grid_name[1].isdigit():
            col_index = ord(grid_name[0].upper()) - ord('A')  # A=0, B=1, C=2, D=3, E=4
            row_index = int(grid_name[1]) - 1  # 1=0, 2=1, 3=2, 4=3, 5=4

            x = self.a1_spawn_position[0] - (row_index * self.grid_spacing)
            y = self.a1_spawn_position[1] + (col_index * self.grid_spacing)
            z = self.a1_spawn_position[2]
            return [x, y, z]

        # Default to A1 if invalid
        return self.a1_spawn_position.copy()

    def calculate_grip_position(self, grid_name: str) -> List[float]:
        """Calculate grip position for a grid name (spawn + grip_offset)"""
        spawn = self._calculate_spawn_position(grid_name)
        return [
            spawn[0] + self.grip_offset[0],
            spawn[1] + self.grip_offset[1],
            spawn[2] + self.grip_offset[2]
        ]

    def is_valid_grid_name(self, grid_name: str) -> bool:
        """Check if a grid name is valid (A1-E5 or special positions like X)"""
        # Check special positions
        if grid_name in self.grid_positions:
            return True

        # Check standard grid (A1-E5)
        if len(grid_name) == 2:
            col = grid_name[0].upper()
            row = grid_name[1]
            if col in 'ABCDE' and row in '12345':
                return True

        return False

    # ========== PUBLIC API FOR PROMPTS ==========

    def parse_location(self, location: str) -> Tuple[str, int]:
        """Parse location string into base position and stack level.

        Examples:
            'A1' -> ('A1', 0)
            'A1:0' -> ('A1', 0)
            'A1:1' -> ('A1', 1)
            'A1:2' -> ('A1', 2)
        """
        if ':' in location:
            parts = location.split(':')
            return parts[0], int(parts[1])
        return location, 0

    def get_current_objects(self) -> List[Dict[str, Any]]:
        """Get list of all current objects with their positions and stack levels"""
        result = []
        for name, data in self.objects.items():
            location = data.get('location')
            if not location:
                continue

            # Parse location to get base position and stack level
            base_pos, stack_level = self.parse_location(location)

            if base_pos in self.grid_positions:
                pos = self.grid_positions[base_pos]
                result.append({
                    'name': name,
                    'location': location,
                    'base_position': base_pos,  # e.g., "A1"
                    'stack_level': stack_level,  # 0, 1, 2, 3
                    'color': data.get('color', 'Grau'),
                    'position': pos,
                    'dimensions': data.get('dimensions', [0.03, 0.03, 0.03])
                })
        return result

    def get_objects_at_position(self, position: str) -> List[Dict[str, Any]]:
        """Get all objects stacked at a specific grid position, sorted by level."""
        objects = []
        for obj in self.get_current_objects():
            if obj['base_position'] == position:
                objects.append(obj)
        # Sort by stack level
        return sorted(objects, key=lambda x: x['stack_level'])

    def get_objects_for_prompt(self) -> str:
        """Get formatted object list for prompts"""
        if 'objects_prompt' in self._cache:
            return self._cache['objects_prompt']

        objects = self.get_current_objects()
        if not objects:
            result = "No cubes currently in the scene."
        else:
            lines = ["Current cubes in the scene:"]
            for obj in objects:
                lines.append(
                    f"  - {obj['name']}: Position {obj['location']}, "
                    f"Color: {obj['color']}, XYZ: ({obj['position'][0]:.3f}, "
                    f"{obj['position'][1]:.3f}, {obj['position'][2]:.3f})"
                )
            result = '\n'.join(lines)

        self._cache['objects_prompt'] = result
        return result

    def get_grid_positions_for_prompt(self) -> str:
        """Get formatted grid positions for prompts"""
        if 'grid_prompt' in self._cache:
            return self._cache['grid_prompt']

        lines = ["Grid Positions (5x5 grid):"]
        lines.append("  Letters (A-E) = Columns (Y-axis)")
        lines.append("  Numbers (1-5) = Rows (X-axis), Row 5 closest to robot")
        lines.append("")

        # Group by row
        rows = {}
        for name, pos in sorted(self.grid_positions.items()):
            if len(name) == 2 and name[0].isalpha() and name[1].isdigit():
                row = name[1]
                if row not in rows:
                    rows[row] = []
                rows[row].append((name, pos))

        for row in sorted(rows.keys(), reverse=True):
            positions = rows[row]
            cols = [f"{name}" for name, _ in sorted(positions)]
            lines.append(f"  Row {row}: {', '.join(cols)}")

        result = '\n'.join(lines)
        self._cache['grid_prompt'] = result
        return result

    def get_occupied_positions(self) -> List[str]:
        """Get list of currently occupied grid positions (base positions only)"""
        occupied = set()
        for obj in self.objects.values():
            loc = obj.get('location')
            if loc:
                base_pos, _ = self.parse_location(loc)
                occupied.add(base_pos)
        return list(occupied)

    def get_free_positions(self) -> List[str]:
        """Get list of free grid positions"""
        occupied = set(self.get_occupied_positions())
        free = []
        for name in self.grid_positions.keys():
            if name not in occupied and len(name) == 2:
                free.append(name)
        return sorted(free)

    def get_object_at_position(self, position: str) -> Optional[Dict[str, Any]]:
        """Get object at a specific grid position"""
        for name, data in self.objects.items():
            if data.get('location') == position:
                return {'name': name, **data}
        return None

    def get_available_colors(self) -> List[str]:
        """Get list of available color names"""
        return ['Schwarz', 'Rot', 'Gruen', 'Gelb', 'Blau', 'Grau', 'Orange']

    def get_prompt_data(self) -> Dict[str, Any]:
        """Get all data needed for prompt generation"""
        return {
            'objects': self.get_current_objects(),
            'objects_text': self.get_objects_for_prompt(),
            'grid_text': self.get_grid_positions_for_prompt(),
            'occupied': self.get_occupied_positions(),
            'free_positions': self.get_free_positions(),
            'colors': self.get_available_colors(),
            'total_positions': len([k for k in self.grid_positions.keys() if len(k) == 2]),
        }


# ========== GLOBAL SINGLETON FUNCTIONS ==========

_parser = None


def get_irb120_parser() -> IRB120AMLParser:
    """Get the global parser instance"""
    global _parser
    if _parser is None:
        _parser = IRB120AMLParser()
    return _parser


def reload_irb120_config():
    """Reload IRB 120 configuration"""
    parser = get_irb120_parser()
    parser.reload()
    print("IRB 120 configuration reloaded")


# ========== TEST FUNCTION ==========

if __name__ == "__main__":
    parser = get_irb120_parser()
    print("\n=== IRB 120 AML Parser Test ===\n")

    print(f"Config file: {parser.config_path}")
    print(f"Grid positions loaded: {len(parser.grid_positions)}")
    print(f"Objects loaded: {len(parser.objects)}")

    print("\n" + parser.get_grid_positions_for_prompt())
    print("\n" + parser.get_objects_for_prompt())

    print(f"\nOccupied positions: {parser.get_occupied_positions()}")
    print(f"Free positions: {parser.get_free_positions()[:10]}...")
