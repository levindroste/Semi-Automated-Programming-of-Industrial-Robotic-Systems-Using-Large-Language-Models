# aml_prompt_parser.py
# Zentraler Parser für AML-Konfiguration und Zustand mit Singleton-Pattern

import xml.etree.ElementTree as ET
import os
from typing import Dict, List, Tuple, Optional, Any
from datetime import datetime


class AMLPromptParser:
    """Singleton Parser für AML-Konfiguration und Zustand"""

    _instance = None
    _initialized = False

    def __new__(cls):
        """Singleton Pattern - nur eine Instanz"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """Initialisiert Parser einmalig"""
        if self._initialized:
            return

        # Pfade
        base = os.path.expanduser("~/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/src/ur10e_hl_interface/config")
        self.config_path = f"{base}/AML-Datei-V04.aml"
        self.state_path = f"{base}/SchaltschrankZustand.aml"

        # Daten
        self.objects = {}
        self.robot_params = {}
        self.rail_info = {}
        self.cabinet_info = {}
        self.current_state = None

        self.ns = {'caex': 'http://www.dke.de/CAEX'}

        # Cache für formatierte Ausgaben
        self._cache = {}

        # Auto-parse bei Initialisierung
        self.reload()
        self._initialized = True

    def reload(self):
        """Lädt Konfiguration und Zustand neu"""
        self._cache.clear()
        self.parse_config()
        self.parse_state()

    # ========== HAUPTMETHODEN FÜR PROMPT-GENERIERUNG ==========

    def get_level1_prompt_data(self) -> Dict[str, Any]:
        """Liefert alle Daten für Level-1-Prompt strukturiert"""
        return {
            'has_state': self.current_state is not None,
            'state_summary': self.get_state_summary(),
            'components_info': self.get_components_for_prompt(),
            'available': self.get_available_components(),
            'rail_info': self.get_rail_info_dict(),
            'color_groups': self.get_color_groups(),
            'size_groups': self.get_size_groups(),
            'mappings': self.get_natural_language_mappings()
        }

    def get_level2_prompt_data(self) -> Dict[str, Any]:
        """Liefert alle Daten für Level-2-Prompt strukturiert"""
        return {
            'has_state': self.current_state is not None,
            'component_enum': self.get_component_enum(),
            'state_load_code': self.get_state_load_code()
        }

    # ========== FORMATIERTE AUSGABEN FÜR PROMPTS ==========

    def get_components_for_prompt(self) -> List[str]:
        """Formatierte Komponentenliste für Prompts"""
        if 'components_prompt' in self._cache:
            return self._cache['components_prompt']

        available = self.get_available_components()
        lines = []

        for name, data in sorted(self.objects.items()):
            lines.append(
                f"- {name}: {data['width']}mm, {data['color']}, "
                f"{data['size_category']}, Available: {available.get(name, 0)}"
            )

        self._cache['components_prompt'] = lines
        return lines

    def get_component_enum(self) -> str:
        """C++ Enum-Mapping für Level-2"""
        if 'component_enum' in self._cache:
            return self._cache['component_enum']

        lines = [f'  "{name}" -> {name}' for name in sorted(self.objects.keys())]
        result = '\n'.join(lines)
        self._cache['component_enum'] = result
        return result

    def get_state_load_code(self) -> str:
        """C++ Code-Snippet für Zustandsladung"""
        if self.current_state:
            return """// Load existing state
    if (!robot->loadStateFromAML(state_file)) {
        RCLCPP_WARN(robot->get_logger(), "No existing state found");
    }
"""
        return "// Fresh start - Rail 1 already at workspace\n"

    def get_color_groups(self) -> Dict[str, List[str]]:
        """Gruppiert Komponenten nach Farbe"""
        if 'color_groups' in self._cache:
            return self._cache['color_groups']

        groups = {}
        for name, data in self.objects.items():
            color = data.get('color', 'Unknown')
            if color not in groups:
                groups[color] = []
            groups[color].append(name)

        self._cache['color_groups'] = groups
        return groups

    def get_size_groups(self) -> Dict[str, List[str]]:
        """Gruppiert Komponenten nach Größe"""
        if 'size_groups' in self._cache:
            return self._cache['size_groups']

        groups = {}
        for name, data in self.objects.items():
            size = data.get('size_category', 'Unknown')
            if size not in groups:
                groups[size] = []
            groups[size].append(name)

        self._cache['size_groups'] = groups
        return groups

    def get_rail_info_dict(self) -> Dict[str, Any]:
        """Rail-Informationen als strukturiertes Dictionary"""
        return {
            'length_m': self.rail_info.get('length', 683) / 1000.0,
            'length_mm': self.rail_info.get('length', 683),
            'spacing_m': float(self.robot_params.get('object_spacing_rail', 10)) / 1000.0,
            'spacing_mm': float(self.robot_params.get('object_spacing_rail', 10)),
            'max_rails': self.rail_info.get('count', 5),
            'available_count': self.get_available_rails()[0],
            'available_ids': self.get_available_rails()[1],
            'workspace_rail': self.current_state.get('workspace_rail', -1) if self.current_state else -1
        }

    def get_formatted_availability(self, show_visual: bool = True) -> str:
        """Formatierte Verfügbarkeitsanzeige mit optionalen visuellen Indikatoren"""
        available = self.get_available_components()
        lines = []

        # Rails
        rail_total = self.rail_info.get('count', 5)
        lines.append(f"Rails:                                     {rail_total}/{rail_total}")

        # Komponenten nach Größe gruppiert
        groups = {}
        size_map = {'klein': 'Small', 'mittel': 'Medium', 'groß': 'Large'}

        for name, data in sorted(self.objects.items()):
            size = data.get('size_category', 'Unknown')
            group = size_map.get(size, size.capitalize())

            if group not in groups:
                groups[group] = []

            avail = available.get(name, data['count'])
            total = data['count']
            color = data.get('color', 'Unknown')

            if show_visual:
                indicator = "✓" if avail > 0 else "✗"
                warning = " ⚠️" if avail == 0 else ""
                display = f"{name} ({data['width']}mm, {color}):"
                padding = ' ' * (35 - len(display))
                line = f"{indicator} {display}{padding}{avail}/{total}{warning}"
            else:
                line = f"{name}: {avail}/{total}"

            groups[group].append(line)

        # Ausgabe nach Gruppen
        for group in ['Small', 'Medium', 'Large']:
            if group in groups and groups[group]:
                lines.append(f"\n{group} components:")
                lines.extend(groups[group])

        return '\n'.join(lines)

    def get_natural_language_mappings(self) -> Dict[str, Any]:
        """Natürlichsprachliche Mappings für Prompts"""
        color_groups = self.get_color_groups()
        size_groups = self.get_size_groups()

        # Farb-Mappings mit Übersetzung
        color_map = {
            'Grau': 'Gray', 'Blau': 'Blue', 'Grün-Gelb': 'Green-Yellow',
            'Rot': 'Red', 'Gelb': 'Yellow', 'Violett': 'Violet/Purple',
            'Orange': 'Orange'
        }

        color_mappings = {}
        for de_color, components in color_groups.items():
            en_color = color_map.get(de_color, de_color)
            color_mappings[f"{de_color} ({en_color})"] = components

        # Größen-Mappings mit Übersetzung
        size_map = {'klein': 'small', 'mittel': 'medium', 'groß': 'large'}

        size_mappings = {}
        for de_size, components in size_groups.items():
            en_size = size_map.get(de_size, de_size)
            size_mappings[f"{de_size} ({en_size})"] = components

        return {
            'colors': color_mappings,
            'sizes': size_mappings,
            'special': {
                'Überspannungsschutz/surge protection': ['VAL_MS_230', 'VAL_MS_T1_T2']
            }
        }

    # ========== BERECHNUNGSMETHODEN ==========

    def get_available_components(self) -> Dict[str, int]:
        """Berechnet verfügbare Komponenten"""
        available = {}
        for name, data in self.objects.items():
            used = 0
            if self.current_state and name in self.current_state.get('used_components', {}):
                used = len(self.current_state['used_components'][name])
            available[name] = data['count'] - used
        return available

    def get_available_rails(self) -> Tuple[int, List[int]]:
        """Berechnet verfügbare Schienen"""
        total = self.rail_info.get('count', 5)
        storage = []

        if self.current_state:
            for i in range(1, total + 1):
                rail = self.current_state['rails'].get(i, {})
                if rail.get('location', 1) == 1:  # Storage
                    storage.append(i)
        else:
            storage = list(range(2, total + 1))  # Rail 1 am Arbeitsplatz

        return len(storage), storage

    def get_state_summary(self) -> str:
        """Generiert Zustandszusammenfassung"""
        if not self.current_state:
            return "No existing state\nStarting fresh configuration"

        lines = ["Current Cabinet State:"]

        # Schaltschrank
        cabinet_rails = []
        for rid, rail in self.current_state['rails'].items():
            if rail.get('location') == 2:  # Cabinet
                cabinet_rails.append((rail.get('cabinet_position', 0), rid, rail))

        if cabinet_rails:
            lines.append("\nRails in cabinet:")
            for pos, rid, rail in sorted(cabinet_rails):
                lines.append(f"- Position {pos}: Rail {rid}")
                for i, comp in enumerate(rail.get('components', []), 1):
                    lines.append(f"  • Pos. {i}: {comp['type']}")
        else:
            lines.append("\nCabinet is empty")

        # Arbeitsplatz
        ws = self.current_state.get('workspace_rail', -1)
        if ws > 0:
            lines.append(f"\nWorkspace: Rail {ws}")
            rail = self.current_state['rails'].get(ws, {})
            for i, comp in enumerate(rail.get('components', []), 1):
                lines.append(f"  • Pos. {i}: {comp['type']}")
        else:
            lines.append("\nWorkspace: Empty")

        return '\n'.join(lines)

    def validate_request(self, requested: Dict[str, int]) -> Tuple[bool, List[str]]:
        """Validiert eine Anfrage gegen verfügbare Komponenten"""
        errors = []
        available = self.get_available_components()

        for comp, count in requested.items():
            if comp not in self.objects:
                errors.append(f"Unknown component: {comp}")
            elif count > available.get(comp, 0):
                errors.append(f"{comp}: Only {available[comp]} available, requested {count}")

        return len(errors) == 0, errors

    # ========== PARSING-METHODEN ==========

    def parse_config(self) -> bool:
        """Lädt Konfiguration aus AML"""
        try:
            tree = ET.parse(self.config_path)
            root = tree.getroot()

            for hierarchy in root.findall('.//caex:InstanceHierarchy', self.ns):
                name = hierarchy.get('Name', '')
                if name == 'Roboter-Parameter':
                    self._parse_params(hierarchy)
                elif name == 'Geometrie-Bibliothek':
                    self._parse_geometry(hierarchy)

            return True
        except Exception as e:
            print(f"Config parse error: {e}")
            return False

    def parse_state(self) -> bool:
        """Lädt Zustand falls vorhanden"""
        if not os.path.exists(self.state_path):
            self.current_state = None
            return False

        try:
            tree = ET.parse(self.state_path)
            self.current_state = {'workspace_rail': -1, 'rails': {}, 'used_components': {}}

            hierarchy = tree.find('.//caex:InstanceHierarchy[@Name="Schaltschrank-Zustand"]', self.ns)
            if hierarchy:
                # Workspace Rail
                info = hierarchy.find('.//caex:InternalElement[@Name="Zustandsinfo"]', self.ns)
                if info:
                    self.current_state['workspace_rail'] = int(self._get_val(info, 'Arbeitsplatz-Schiene', '-1'))

                # Rails
                rails = hierarchy.find('.//caex:InternalElement[@Name="Hutschienen"]', self.ns)
                if rails:
                    for rail in rails.findall('.//caex:InternalElement', self.ns):
                        self._parse_rail_state(rail)

            return True
        except Exception as e:
            print(f"State parse error: {e}")
            self.current_state = None
            return False

    # ========== PRIVATE HELPER-METHODEN ==========

    def _parse_params(self, hierarchy: ET.Element):
        """Parst Roboter-Parameter"""
        for elem in hierarchy.findall('.//caex:InternalElement', self.ns):
            name = elem.get('Name', '')

            if name == 'Greif-Parameter':
                self.robot_params = {
                    'approach_height': self._get_val(elem, 'Approach-Höhe'),
                    'object_x_spacing': self._get_val(elem, 'Objekt-X-Spacing'),
                    'object_spacing_rail': self._get_val(elem, 'Objekt-Spacing-auf-Schiene'),
                    'standard_pick_orientation': self._get_val(elem, 'Standard-Pick-Orientierung'),
                    'standard_place_orientation': self._get_val(elem, 'Standard-Place-Orientierung')
                }

            elif name == 'Schaltschrank-Parameter':
                self.cabinet_info = {
                    'base_x': self._get_val(elem, 'Basis-Position-X'),
                    'base_y': self._get_val(elem, 'Basis-Position-Y'),
                    'base_z': self._get_val(elem, 'Basis-Position-Z'),
                    'rail_spacing': self._get_val(elem, 'Schienen-Abstand'),
                    'hutschienen_lager_abstand': self._get_val(elem, 'Hutschienen-Lager-Abstand')
                }

    def _parse_geometry(self, hierarchy: ET.Element):
        """Parst Komponenten aus Geometrie-Bibliothek"""
        bauteile = hierarchy.find('.//caex:InternalElement[@Name="Bauteile"]', self.ns)
        if not bauteile:
            return

        excluded = {'Schaltschrank', 'Hutschienen-Halter', 'LPS-Logo', 'Grundplatte', 'Montageblock'}

        for elem in bauteile.findall('.//caex:InternalElement', self.ns):
            name = elem.get('Name', '')

            if name == 'Hutschiene':
                self.rail_info = {
                    'name': 'Hutschiene',
                    'description': self._get_val(elem, 'Basics/Beschreibung'),
                    'length': float(self._get_val(elem, 'Geometrie/Länge', '683')),
                    'count': int(self._get_val(elem, 'Roboter-Daten/Anzahl', '5')),
                    'start_position': self._get_val(elem, 'Roboter-Daten/Start-Position'),
                    'end_position': self._get_val(elem, 'Roboter-Daten/End-Position'),
                    'orientation': self._get_val(elem, 'Roboter-Daten/Greif-Orientierung'),
                    'y_spacing': float(self.cabinet_info.get('hutschienen_lager_abstand', '0.1'))
                }

            elif name and name not in excluded:
                self.objects[name] = {
                    'name': name,
                    'description': self._get_val(elem, 'Basics/Beschreibung'),
                    'color': self._get_val(elem, 'Basics/Farbe'),
                    'size_category': self._get_val(elem, 'Basics/Größenkategorie'),
                    'width': float(self._get_val(elem, 'Geometrie/Breite', '10')),
                    'count': int(self._get_val(elem, 'Roboter-Daten/Anzahl', '1')),
                    'pick_position': self._get_val(elem, 'Roboter-Daten/Pick-Position')
                }

    def _parse_rail_state(self, rail_elem: ET.Element):
        """Parst Schienenzustand"""
        name = rail_elem.get('Name', '')
        if not name.startswith('Schiene-'):
            return

        rail_id = int(name.split('-')[1])
        state = {
            'location': int(self._get_val(rail_elem, 'Location', '0')),
            'cabinet_position': int(self._get_val(rail_elem, 'Cabinet-Position', '-1')),
            'fill_level': float(self._get_val(rail_elem, 'Füllstand', '0')),
            'components': []
        }

        # Komponenten
        comps = rail_elem.find('.//caex:InternalElement[@Name="Komponenten"]', self.ns)
        if comps:
            for comp in comps.findall('.//caex:InternalElement', self.ns):
                comp_type = self._get_val(comp, 'Komponenten-Typ')
                instance = int(self._get_val(comp, 'Instance-Number', '0'))

                state['components'].append({
                    'type': comp_type,
                    'instance': instance,
                    'position': float(self._get_val(comp, 'Position-auf-Schiene', '0'))
                })

                # Track verwendete Komponenten
                if comp_type not in self.current_state['used_components']:
                    self.current_state['used_components'][comp_type] = []
                self.current_state['used_components'][comp_type].append(instance)

        self.current_state['rails'][rail_id] = state

    def _get_val(self, elem: ET.Element, path: str, default: str = "") -> str:
        """Holt Wert aus XML-Element"""
        parts = path.split('/')
        current = elem

        for i, part in enumerate(parts):
            if i == len(parts) - 1:
                for attr in current.findall(f'.//caex:Attribute[@Name="{part}"]', self.ns):
                    val = attr.find('.//caex:Value', self.ns)
                    if val is not None and val.text:
                        return val.text.strip()
            else:
                found = False
                for attr in current.findall(f'.//caex:Attribute[@Name="{part}"]', self.ns):
                    current = attr
                    found = True
                    break
                if not found:
                    return default
        return default

    def _add_attr(self, parent: ET.Element, name: str, dtype: str, value: str):
        """Fügt XML-Attribut hinzu"""
        attr = ET.SubElement(parent, 'Attribute', {'Name': name, 'AttributeDataType': dtype})
        ET.SubElement(attr, 'Value').text = value

    # ========== ZUSTANDSSPEICHERUNG ==========

    def save_state(self, state_data: Dict[str, Any]) -> bool:
        """Speichert Zustand in AML-Datei"""
        try:
            # XML erstellen
            root = ET.Element('CAEXFile', {
                'SchemaVersion': '3.0',
                'FileName': 'SchaltschrankZustand.aml',
                'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                'xmlns': 'http://www.dke.de/CAEX'
            })

            ET.SubElement(root, 'SuperiorStandardVersion').text = 'AutomationML 2.1'

            # Hierarchie
            hierarchy = ET.SubElement(root, 'InstanceHierarchy', {
                'Name': 'Schaltschrank-Zustand',
                'ID': 'cabinet-state'
            })

            # Zustandsinfo
            info = ET.SubElement(hierarchy, 'InternalElement', {
                'Name': 'Zustandsinfo',
                'ID': 'state-info'
            })
            self._add_attr(info, 'Letztes-Update', 'xs:dateTime', datetime.now().isoformat())
            self._add_attr(info, 'Arbeitsplatz-Schiene', 'xs:int', str(state_data.get('workspace_rail_id', -1)))

            # Rails
            rails = ET.SubElement(hierarchy, 'InternalElement', {
                'Name': 'Hutschienen',
                'ID': 'rails-state'
            })

            for rail_id, rail_state in state_data.get('rail_states', {}).items():
                rail = ET.SubElement(rails, 'InternalElement', {
                    'Name': f'Schiene-{rail_id}',
                    'ID': f'rail-{rail_id}'
                })

                self._add_attr(rail, 'Location', 'xs:string', str(rail_state.get('location', 0)))
                self._add_attr(rail, 'Cabinet-Position', 'xs:int', str(rail_state.get('cabinet_position', -1)))
                self._add_attr(rail, 'Füllstand', 'xs:double', str(rail_state.get('fill_level', 0.0)))

                # Komponenten
                if rail_state.get('components'):
                    comps = ET.SubElement(rail, 'InternalElement', {
                        'Name': 'Komponenten',
                        'ID': f'rail-{rail_id}-components'
                    })

                    for i, comp in enumerate(rail_state['components'], 1):
                        c = ET.SubElement(comps, 'InternalElement', {
                            'Name': f'Position-{i}',
                            'ID': f'rail-{rail_id}-pos-{i}'
                        })

                        self._add_attr(c, 'Komponenten-Typ', 'xs:string', comp['type_name'])
                        self._add_attr(c, 'Original-ID', 'xs:string', comp['object_id'])
                        self._add_attr(c, 'Instance-Number', 'xs:int', str(comp['instance_number']))
                        self._add_attr(c, 'Position-auf-Schiene', 'xs:double', str(comp['position_on_rail']))
                        self._add_attr(c, 'Breite', 'xs:double', str(comp['width']))

            # Speichern
            tree = ET.ElementTree(root)
            ET.indent(tree, space='  ')
            tree.write(self.state_path, encoding='utf-8', xml_declaration=True)
            return True

        except Exception as e:
            print(f"Save error: {e}")
            return False


# ========== GLOBALE SINGLETON-FUNKTIONEN ==========

_parser = None


def get_parser() -> AMLPromptParser:
    """Liefert die globale Parser-Instanz"""
    global _parser
    if _parser is None:
        _parser = AMLPromptParser()
    return _parser


# ========== KOMPATIBILITÄTS-WRAPPER ==========

class AMLStateManager:
    """Wrapper für alte API-Kompatibilität"""

    def __init__(self):
        self.parser = get_parser()

    def save_state(self, path: str, data: Dict[str, Any]) -> bool:
        old_path = self.parser.state_path
        self.parser.state_path = path
        result = self.parser.save_state(data)
        self.parser.state_path = old_path
        return result

    def load_state(self, path: str) -> Optional[Dict[str, Any]]:
        old_path = self.parser.state_path
        self.parser.state_path = path
        result = self.parser.current_state if self.parser.parse_state() else None
        self.parser.state_path = old_path
        return result