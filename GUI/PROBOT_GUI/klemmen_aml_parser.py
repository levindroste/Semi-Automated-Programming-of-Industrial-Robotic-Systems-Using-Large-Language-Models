# klemmen_aml_parser.py
# Parser for Klemmen-Modus (Terminal Block Mode) configuration
# Reads clamp positions, cartons, and packages for dynamic prompt generation
# Includes position tracking for calculating pick positions

import xml.etree.ElementTree as ET
import os
from typing import Dict, List, Tuple, Optional, Any


class KlemmenAMLParser:
    """Parser for Klemmen-Modus AML configuration with position tracking"""

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
        self.config_path = f"{base}/klemmen_config.aml"

        # Data storage
        self.klemmen = {}      # Klemme name -> {breite, basis_pos, x_offset, anzahl_pro_reihe, basis_pos_reihe2, gripping_position, gripping_x_offset}
        self.kartons = {}      # Karton name -> {position}
        self.pakete = {}       # Paket name -> {inhalt: {klemme_name: count}}

        # Robot config (includes central gripping orientation for all Klemmen)
        self.robot_config = {}
        self.klemmen_gripping_orientation = [0.0, 0.7, 0.7, 0.0]  # Default quaternion

        # Position tracking - how many of each clamp type have been picked
        self.picked_counts = {}  # Klemme name -> number picked

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
        self.klemmen.clear()
        self.kartons.clear()
        self.pakete.clear()
        self.robot_config.clear()
        # Keep picked_counts during reload (they represent current state)
        self.parse_config()

    def reset_picked_counts(self):
        """Reset all picked counts to zero (start fresh)"""
        self.picked_counts = {name: 0 for name in self.klemmen.keys()}
        self._cache.clear()

    def parse_config(self) -> bool:
        """Parse the AML configuration file"""
        if not os.path.exists(self.config_path):
            print(f"Klemmen AML config not found: {self.config_path}")
            return False

        try:
            tree = ET.parse(self.config_path)
            root = tree.getroot()

            # Parse hierarchies
            for hierarchy in root.findall('.//aml:InstanceHierarchy', self.ns):
                name = hierarchy.get('Name', '')
                if name == 'Robot-Config':
                    self._parse_robot_config(hierarchy)
                elif name == 'Klemmen':
                    self._parse_klemmen(hierarchy)
                elif name == 'Kartons':
                    self._parse_kartons(hierarchy)
                elif name == 'Pakete':
                    self._parse_pakete(hierarchy)

            # Initialize picked counts for new klemmen
            for klemme_name in self.klemmen.keys():
                if klemme_name not in self.picked_counts:
                    self.picked_counts[klemme_name] = 0

            return True

        except ET.ParseError as e:
            print(f"XML parse error: {e}")
            return False
        except Exception as e:
            print(f"Config parse error: {e}")
            return False

    def _parse_robot_config(self, hierarchy: ET.Element):
        """Parse Robot-Config hierarchy for central settings"""
        for elem in hierarchy.findall('aml:InternalElement', self.ns):
            for attr in elem.findall('aml:Attribute', self.ns):
                attr_name = attr.get('Name')
                value_elem = attr.find('aml:Value', self.ns)
                if value_elem is None or not value_elem.text:
                    continue

                try:
                    if attr_name == 'KlemmenGrippingOrientation':
                        parts = value_elem.text.split(',')
                        if len(parts) >= 4:
                            self.klemmen_gripping_orientation = [float(p) for p in parts[:4]]
                    elif attr_name == 'ApproachHeight':
                        self.robot_config['approach_height'] = float(value_elem.text)
                    elif attr_name == 'VelocityScaling':
                        self.robot_config['velocity_scaling'] = float(value_elem.text)
                    elif attr_name == 'AccelerationScaling':
                        self.robot_config['acceleration_scaling'] = float(value_elem.text)
                except ValueError:
                    pass

    def _parse_klemmen(self, hierarchy: ET.Element):
        """Parse Klemmen hierarchy"""
        for elem in hierarchy.findall('aml:InternalElement', self.ns):
            name = elem.get('Name')
            if not name:
                continue

            klemme_data = {
                'name': name,
                'breite': 10,  # Default 10mm
                'anzahl': 0,   # Actual number of physical clamps
                'basis_position': [0.0, 0.0, 0.0],
                'x_offset': 0.015,
                'anzahl_pro_reihe': 6,
                'basis_position_reihe2': None,  # Optional second row
                'gripping_position': None,  # Real robot gripping position
                'gripping_x_offset': None   # X offset between clamps of same type
            }

            for attr in elem.findall('aml:Attribute', self.ns):
                attr_name = attr.get('Name')
                value_elem = attr.find('aml:Value', self.ns)
                if value_elem is None or not value_elem.text:
                    continue

                try:
                    if attr_name == 'Breite':
                        klemme_data['breite'] = int(value_elem.text)
                    elif attr_name == 'Anzahl':
                        klemme_data['anzahl'] = int(value_elem.text)
                    elif attr_name == 'BasisPosition':
                        parts = value_elem.text.split(',')
                        if len(parts) >= 3:
                            klemme_data['basis_position'] = [float(p) for p in parts[:3]]
                    elif attr_name == 'XOffset':
                        klemme_data['x_offset'] = float(value_elem.text)
                    elif attr_name == 'AnzahlProReihe':
                        klemme_data['anzahl_pro_reihe'] = int(value_elem.text)
                    elif attr_name == 'BasisPositionReihe2':
                        parts = value_elem.text.split(',')
                        if len(parts) >= 3:
                            klemme_data['basis_position_reihe2'] = [float(p) for p in parts[:3]]
                    elif attr_name == 'GrippingPosition':
                        parts = value_elem.text.split(',')
                        if len(parts) >= 3:
                            klemme_data['gripping_position'] = [float(p) for p in parts[:3]]
                    elif attr_name == 'GrippingXOffset':
                        klemme_data['gripping_x_offset'] = float(value_elem.text)
                except ValueError:
                    pass

            self.klemmen[name] = klemme_data

    def _parse_kartons(self, hierarchy: ET.Element):
        """Parse Kartons hierarchy"""
        for elem in hierarchy.findall('aml:InternalElement', self.ns):
            name = elem.get('Name')
            if not name:
                continue

            karton_data = {
                'name': name,
                'position': [0.0, 0.0, 0.0]
            }

            for attr in elem.findall('aml:Attribute', self.ns):
                attr_name = attr.get('Name')
                value_elem = attr.find('aml:Value', self.ns)
                if value_elem is None or not value_elem.text:
                    continue

                if attr_name == 'Position':
                    try:
                        parts = value_elem.text.split(',')
                        if len(parts) >= 3:
                            karton_data['position'] = [float(p) for p in parts[:3]]
                    except ValueError:
                        pass

            self.kartons[name] = karton_data

    def _parse_pakete(self, hierarchy: ET.Element):
        """Parse Pakete hierarchy"""
        for elem in hierarchy.findall('aml:InternalElement', self.ns):
            name = elem.get('Name')
            if not name:
                continue

            paket_data = {
                'name': name,
                'inhalt': {}  # {klemme_name: count}
            }

            for attr in elem.findall('aml:Attribute', self.ns):
                attr_name = attr.get('Name')
                value_elem = attr.find('aml:Value', self.ns)
                if value_elem is None or not value_elem.text:
                    continue

                if attr_name == 'Inhalt':
                    # Parse "Klemme1:2,Klemme3:1,Klemme5:1"
                    try:
                        items = value_elem.text.split(',')
                        for item in items:
                            parts = item.strip().split(':')
                            if len(parts) == 2:
                                klemme_name = parts[0].strip()
                                count = int(parts[1].strip())
                                paket_data['inhalt'][klemme_name] = count
                    except ValueError:
                        pass

            self.pakete[name] = paket_data

    # ========== POSITION CALCULATION ==========

    def get_total_available(self, klemme_name: str) -> int:
        """Get total available clamps for a type (from Anzahl attribute in AML)"""
        klemme = self.klemmen.get(klemme_name)
        if not klemme:
            return 0

        return klemme['anzahl']

    def get_remaining_available(self, klemme_name: str) -> int:
        """Get remaining available clamps for a type"""
        total = self.get_total_available(klemme_name)
        picked = self.picked_counts.get(klemme_name, 0)
        return max(0, total - picked)

    def calculate_pick_position(self, klemme_name: str) -> Optional[List[float]]:
        """
        Calculate the current pick position for a clamp type.
        Returns None if no more clamps available.

        Position calculation:
        - Row 1: basis_position + (picked_count % anzahl_pro_reihe) * x_offset
        - Row 2 (if exists and row 1 empty):
          basis_position_reihe2 + ((picked_count - anzahl_pro_reihe) % anzahl_pro_reihe) * x_offset
        """
        klemme = self.klemmen.get(klemme_name)
        if not klemme:
            print(f"Unknown Klemme: {klemme_name}")
            return None

        picked = self.picked_counts.get(klemme_name, 0)
        anzahl = klemme['anzahl_pro_reihe']
        total = self.get_total_available(klemme_name)

        if picked >= total:
            print(f"No more {klemme_name} available (picked {picked}/{total})")
            return None

        # Determine which row and position within row
        if picked < anzahl:
            # Row 1
            base = klemme['basis_position'].copy()
            offset_count = picked
        else:
            # Row 2
            if klemme['basis_position_reihe2'] is None:
                return None
            base = klemme['basis_position_reihe2'].copy()
            offset_count = picked - anzahl

        # Apply X offset (offset is in X direction)
        position = base.copy()
        position[0] += offset_count * klemme['x_offset']

        return position

    def pick_klemme(self, klemme_name: str) -> Optional[List[float]]:
        """
        Get pick position and increment picked count.
        Returns the position to pick from, or None if unavailable.
        """
        position = self.calculate_pick_position(klemme_name)
        if position is not None:
            self.picked_counts[klemme_name] = self.picked_counts.get(klemme_name, 0) + 1
            self._cache.clear()  # Clear cache after state change
        return position

    def get_karton_position(self, karton_name: str) -> Optional[List[float]]:
        """Get position of a carton"""
        karton = self.kartons.get(karton_name)
        if karton:
            return karton['position'].copy()
        return None

    def expand_paket(self, paket_name: str) -> Dict[str, int]:
        """Expand a package to its clamp contents"""
        paket = self.pakete.get(paket_name)
        if paket:
            return paket['inhalt'].copy()
        return {}

    # ========== PUBLIC API FOR PROMPTS ==========

    def get_gripping_orientation(self) -> List[float]:
        """Get the central gripping orientation (quaternion) for all Klemmen"""
        return self.klemmen_gripping_orientation.copy()

    def get_gripping_position(self, klemme_name: str, index: int = 0) -> Optional[List[float]]:
        """
        Get the real robot gripping position for a specific clamp.

        Args:
            klemme_name: Name of the clamp type (e.g., 'Klemme1')
            index: Which clamp of this type (0 = first, 1 = second, etc.)

        Returns:
            [x, y, z] position in meters, or None if not configured
        """
        klemme = self.klemmen.get(klemme_name)
        if not klemme or klemme['gripping_position'] is None:
            return None

        base_pos = klemme['gripping_position'].copy()
        x_offset = klemme.get('gripping_x_offset', 0.0)

        if x_offset and index > 0:
            base_pos[0] += index * x_offset

        return base_pos

    def get_klemmen_for_prompt(self) -> str:
        """Get formatted clamp list for prompts"""
        if 'klemmen_prompt' in self._cache:
            return self._cache['klemmen_prompt']

        if not self.klemmen:
            result = "Keine Klemmen konfiguriert."
        else:
            lines = ["Verfügbare Klemmen (alle grau):"]
            for name, data in sorted(self.klemmen.items()):
                total = self.get_total_available(name)
                remaining = self.get_remaining_available(name)
                lines.append(
                    f"  - {name}: {remaining}/{total} verfügbar "
                    f"(Breite: {data['breite']}mm)"
                )
            result = '\n'.join(lines)

        self._cache['klemmen_prompt'] = result
        return result

    def get_klemmen_table_for_prompt(self) -> str:
        """Get formatted clamp availability table for prompts"""
        if 'klemmen_table' in self._cache:
            return self._cache['klemmen_table']

        if not self.klemmen:
            result = "Keine Klemmen konfiguriert."
        else:
            lines = [
                "| Klemmentyp | Verfügbar | Max |",
                "|------------|-----------|-----|"
            ]
            for name, data in sorted(self.klemmen.items()):
                total = self.get_total_available(name)
                remaining = self.get_remaining_available(name)
                lines.append(f"| {name:10} | {remaining:9} | {total:3} |")
            result = '\n'.join(lines)

        self._cache['klemmen_table'] = result
        return result

    def get_kartons_for_prompt(self) -> str:
        """Get formatted carton list for prompts"""
        if 'kartons_prompt' in self._cache:
            return self._cache['kartons_prompt']

        if not self.kartons:
            result = "Keine Kartons konfiguriert."
        else:
            lines = ["Verfügbare Kartons:"]
            for name, data in sorted(self.kartons.items()):
                pos = data['position']
                lines.append(
                    f"  - {name}: Position ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
                )
            result = '\n'.join(lines)

        self._cache['kartons_prompt'] = result
        return result

    def get_pakete_for_prompt(self) -> str:
        """Get formatted package list for prompts"""
        if 'pakete_prompt' in self._cache:
            return self._cache['pakete_prompt']

        if not self.pakete:
            result = "Keine Pakete konfiguriert."
        else:
            lines = ["Definierte Pakete:"]
            for name, data in sorted(self.pakete.items()):
                inhalt_str = ", ".join(
                    f"{k}:{v}" for k, v in sorted(data['inhalt'].items())
                )
                lines.append(f"  - {name}: {inhalt_str}")
            result = '\n'.join(lines)

        self._cache['pakete_prompt'] = result
        return result

    def get_prompt_data(self) -> Dict[str, Any]:
        """Get all data needed for prompt generation"""
        return {
            'klemmen': list(self.klemmen.values()),
            'klemmen_text': self.get_klemmen_for_prompt(),
            'klemmen_table': self.get_klemmen_table_for_prompt(),
            'kartons': list(self.kartons.values()),
            'kartons_text': self.get_kartons_for_prompt(),
            'karton_names': list(self.kartons.keys()),
            'pakete': self.pakete,
            'pakete_text': self.get_pakete_for_prompt(),
            'paket_names': list(self.pakete.keys()),
            'picked_counts': self.picked_counts.copy(),
            'gripping_orientation': self.get_gripping_orientation(),
            'robot_config': self.robot_config.copy(),
        }

    def get_klemmen_status(self) -> List[Dict[str, Any]]:
        """Get status of all clamps for GUI display"""
        status = []
        for name, data in sorted(self.klemmen.items()):
            total = self.get_total_available(name)
            remaining = self.get_remaining_available(name)
            status.append({
                'name': name,
                'breite': data['breite'],
                'total': total,
                'remaining': remaining,
                'picked': self.picked_counts.get(name, 0)
            })
        return status

    def check_availability(self, requirements: Dict[str, int]) -> Tuple[bool, str]:
        """
        Check if the required clamps are available.

        Args:
            requirements: Dict of {klemme_name: count_needed}

        Returns:
            (success, message) tuple
        """
        missing = []
        for klemme_name, count_needed in requirements.items():
            if klemme_name not in self.klemmen:
                missing.append(f"{klemme_name}: unbekannt")
                continue

            remaining = self.get_remaining_available(klemme_name)
            if remaining < count_needed:
                missing.append(
                    f"{klemme_name}: benötigt {count_needed}, verfügbar {remaining}"
                )

        if missing:
            return False, "Nicht genug Klemmen verfügbar:\n" + "\n".join(missing)
        return True, "Alle Klemmen verfügbar"


# ========== GLOBAL SINGLETON FUNCTIONS ==========

_parser = None


def get_klemmen_parser() -> KlemmenAMLParser:
    """Get the global parser instance for Klemmen-Modus"""
    global _parser
    if _parser is None:
        _parser = KlemmenAMLParser()
    return _parser


def reload_klemmen_config():
    """Reload Klemmen-Modus configuration"""
    parser = get_klemmen_parser()
    parser.reload()
    print("Klemmen configuration reloaded")


def reset_klemmen_state():
    """Reset all picked counts (start fresh)"""
    parser = get_klemmen_parser()
    parser.reset_picked_counts()
    print("Klemmen state reset")


# ========== TEST FUNCTION ==========

if __name__ == "__main__":
    parser = get_klemmen_parser()
    print("\n=== Klemmen AML Parser Test ===\n")

    print(f"Config file: {parser.config_path}")
    print(f"Klemmen loaded: {len(parser.klemmen)}")
    print(f"Kartons loaded: {len(parser.kartons)}")
    print(f"Pakete loaded: {len(parser.pakete)}")

    # Test central gripping orientation
    print("\n=== Central Gripping Orientation ===")
    orientation = parser.get_gripping_orientation()
    print(f"Quaternion: [{orientation[0]:.4f}, {orientation[1]:.4f}, {orientation[2]:.4f}, {orientation[3]:.4f}]")

    # Test gripping positions
    print("\n=== Gripping Positions ===")
    for klemme_name in sorted(parser.klemmen.keys()):
        pos = parser.get_gripping_position(klemme_name, 0)
        if pos:
            print(f"{klemme_name}: ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})")
        else:
            print(f"{klemme_name}: not configured")

    print("\n" + parser.get_klemmen_for_prompt())
    print("\n" + parser.get_kartons_for_prompt())
    print("\n" + parser.get_pakete_for_prompt())

    # Test position calculation
    print("\n=== Position Calculation Test ===")
    for i in range(3):
        pos = parser.pick_klemme("Klemme1")
        if pos:
            print(f"Klemme1 pick {i+1}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

    print(f"\nKlemme1 remaining: {parser.get_remaining_available('Klemme1')}")

    # Test availability check
    print("\n=== Availability Check Test ===")
    ok, msg = parser.check_availability({"Klemme1": 5, "Klemme2": 20})
    print(f"Check result: {ok}")
    print(f"Message: {msg}")
