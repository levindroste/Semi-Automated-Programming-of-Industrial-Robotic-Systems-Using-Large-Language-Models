# aml_parser.py
# Parser für AML-Dateien für Robot High-Level Interface

import xml.etree.ElementTree as ET
import os
from typing import Dict, List, Any, Tuple, Optional
from datetime import datetime


class AMLParser:
    """Parser für AML-Konfigurationsdateien"""

    def __init__(self, aml_file_path: str = None):
        """Initialisiert Parser mit AML-Pfad"""
        self.aml_file_path = aml_file_path or os.path.expanduser(
            "~/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/src/ur10e_hl_interface/config/AML-Datei-V04.aml"
        )
        self.objects = {}
        self.robot_params = {}
        self.rail_info = {}
        self.cabinet_info = {}

    def parse(self) -> bool:
        """Parst AML-Datei und extrahiert alle Daten"""
        try:
            tree = ET.parse(self.aml_file_path)
            root = tree.getroot()
            ns = {'caex': 'http://www.dke.de/CAEX'}

            # Parse Hierarchien
            for hierarchy in root.findall('.//caex:InstanceHierarchy', ns):
                name = hierarchy.get('Name')
                if name == 'Roboter-Parameter':
                    self._parse_robot_parameters(hierarchy, ns)
                elif name == 'Geometrie-Bibliothek':
                    self._parse_objects(hierarchy, ns)

            return True
        except Exception as e:
            print(f"Fehler beim Parsen: {e}")
            return False

    def _parse_robot_parameters(self, hierarchy: ET.Element, ns: dict):
        """Extrahiert Roboter-Parameter"""
        for elem in hierarchy.findall('.//caex:InternalElement', ns):
            elem_name = elem.get('Name')

            if elem_name == 'Greif-Parameter':
                self.robot_params = {
                    'approach_height': self._get_value(elem, 'Approach-Höhe', ns),
                    # Place-Höhen-Offset entfernt (war 0)
                    'object_x_spacing': self._get_value(elem, 'Objekt-X-Spacing', ns),
                    'object_spacing_rail': self._get_value(elem, 'Objekt-Spacing-auf-Schiene', ns),
                    'standard_pick_orientation': self._get_value(elem, 'Standard-Pick-Orientierung', ns),
                    'standard_place_orientation': self._get_value(elem, 'Standard-Place-Orientierung', ns)
                }

            elif elem_name == 'Schaltschrank-Parameter':
                self.cabinet_info = {
                    'base_x': self._get_value(elem, 'Basis-Position-X', ns),
                    'base_y': self._get_value(elem, 'Basis-Position-Y', ns),
                    'base_z': self._get_value(elem, 'Basis-Position-Z', ns),
                    'rail_spacing': self._get_value(elem, 'Schienen-Abstand', ns),
                    'hutschienen_lager_abstand': self._get_value(elem, 'Hutschienen-Lager-Abstand', ns)
                }

    def _parse_objects(self, hierarchy: ET.Element, ns: dict):
        """Extrahiert Objekt-Definitionen"""
        bauteile = hierarchy.find('.//caex:InternalElement[@Name="Bauteile"]', ns)
        if not bauteile:
            return

        excluded = ['Schaltschrank', 'Hutschienen-Halter', 'LPS-Logo', 'Grundplatte', 'Montageblock']

        for bauteil in bauteile.findall('.//caex:InternalElement', ns):
            name = bauteil.get('Name')

            if name == 'Hutschiene':
                self._parse_rail_data(bauteil, ns)
            elif name and name not in excluded:
                self.objects[name] = self._parse_object_data(bauteil, ns)

    def _parse_rail_data(self, element: ET.Element, ns: dict):
        """Extrahiert Hutschienen-Daten"""
        self.rail_info = {
            'name': 'Hutschiene',
            'description': self._get_value(element, 'Basics/Beschreibung', ns),
            'length': float(self._get_value(element, 'Geometrie/Länge', ns) or '683'),
            'start_position': self._get_value(element, 'Roboter-Daten/Start-Position', ns),
            'end_position': self._get_value(element, 'Roboter-Daten/End-Position', ns),
            'orientation': self._get_value(element, 'Roboter-Daten/Greif-Orientierung', ns),
            'count': int(self._get_value(element, 'Roboter-Daten/Anzahl', ns) or '3'),
            'spawn_base_position': self._get_value(element, 'Spawn-Daten/Basis-Position', ns),
            # Verwende globalen Wert statt lokalem Y-Spacing
            'y_spacing': float(self.cabinet_info.get('hutschienen_lager_abstand', '0.1'))
        }

    def _parse_object_data(self, element: ET.Element, ns: dict) -> Dict[str, Any]:
        """Extrahiert Daten eines Objekts"""
        return {
            'name': element.get('Name'),
            'description': self._get_value(element, 'Basics/Beschreibung', ns),
            'color': self._get_value(element, 'Basics/Farbe', ns),
            'size_category': self._get_value(element, 'Basics/Größenkategorie', ns),
            'width': float(self._get_value(element, 'Geometrie/Breite', ns) or '10'),
            # Dimensionen-String entfernt (nur Breite wird benötigt)
            'pick_position': self._get_value(element, 'Roboter-Daten/Pick-Position', ns),
            'count': int(self._get_value(element, 'Roboter-Daten/Anzahl', ns) or '1'),
            'object_x_spacing': float(self.robot_params.get('object_x_spacing', '200'))
        }

    def _get_value(self, element: ET.Element, path: str, ns: dict) -> Optional[str]:
        """Holt Attributwert aus verschachtelter XML-Struktur"""
        parts = path.split('/')
        current = element

        for i, part in enumerate(parts):
            if i == len(parts) - 1:
                # Letzter Teil - Attribut suchen
                for attr in current.findall(f'.//caex:Attribute[@Name="{part}"]', ns):
                    value = attr.find('.//caex:Value', ns)
                    if value is not None and value.text:
                        return value.text.strip()
            else:
                # Navigation zum nächsten Container
                found = False
                for attr in current.findall(f'.//caex:Attribute[@Name="{part}"]', ns):
                    current = attr
                    found = True
                    break
                if not found:
                    return None
        return None

    def get_config_for_cpp(self) -> Dict[str, Any]:
        """Gibt Konfiguration für C++ zurück"""
        return {
            'robot_params': self.robot_params,
            'cabinet_info': self.cabinet_info,
            'rail_info': self.rail_info,
            'objects': self.objects
        }

    def validate_config(self) -> Tuple[bool, List[str]]:
        """Validiert die geladene Konfiguration"""
        errors = []

        # Prüfe Parameter
        if not self.robot_params:
            errors.append("Keine Roboter-Parameter gefunden")
        if not self.cabinet_info:
            errors.append("Keine Schaltschrank-Info gefunden")
        if not self.rail_info:
            errors.append("Keine Hutschienen-Info gefunden")
        if not self.objects:
            errors.append("Keine Objekte gefunden")

        # Prüfe ob alle Objekte Farbe und Größenkategorie haben
        for name, obj in self.objects.items():
            if not obj.get('color'):
                errors.append(f"{name}: Keine Farbe definiert")
            if not obj.get('size_category'):
                errors.append(f"{name}: Keine Größenkategorie definiert")

        return len(errors) == 0, errors


class AMLStateManager:
    """Manager für Zustandsspeicherung"""

    def __init__(self):
        self.ns = {'caex': 'http://www.dke.de/CAEX'}

    def save_state(self, state_file_path: str, state_data: Dict[str, Any]) -> bool:
        """Speichert Zustand in AML-Datei"""
        try:
            # XML-Root erstellen
            root = ET.Element('CAEXFile', {
                'SchemaVersion': '3.0',
                'FileName': 'SchaltschrankZustand.aml',
                'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                'xmlns': 'http://www.dke.de/CAEX'
            })

            ET.SubElement(root, 'SuperiorStandardVersion').text = 'AutomationML 2.1'

            # Hierarchie erstellen
            hierarchy = ET.SubElement(root, 'InstanceHierarchy', {
                'Name': 'Schaltschrank-Zustand',
                'ID': 'cabinet-state'
            })

            # Zustandsinfo
            state_info = ET.SubElement(hierarchy, 'InternalElement', {
                'Name': 'Zustandsinfo',
                'ID': 'state-info'
            })

            self._add_attr(state_info, 'Letztes-Update', 'xs:dateTime', datetime.now().isoformat())
            self._add_attr(state_info, 'Arbeitsplatz-Schiene', 'xs:int',
                           str(state_data.get('workspace_rail_id', -1)))

            # Schienen speichern
            rails_elem = ET.SubElement(hierarchy, 'InternalElement', {
                'Name': 'Hutschienen',
                'ID': 'rails-state'
            })

            for rail_id, rail_state in state_data.get('rail_states', {}).items():
                rail_elem = ET.SubElement(rails_elem, 'InternalElement', {
                    'Name': f'Schiene-{rail_id}',
                    'ID': f'rail-{rail_id}'
                })

                self._add_attr(rail_elem, 'Location', 'xs:string',
                               str(rail_state['location']))
                self._add_attr(rail_elem, 'Cabinet-Position', 'xs:int',
                               str(rail_state.get('cabinet_position', -1)))
                self._add_attr(rail_elem, 'Füllstand', 'xs:double',
                               str(rail_state.get('fill_level', 0.0)))

                # Komponenten speichern
                if rail_state.get('components'):
                    comp_elem = ET.SubElement(rail_elem, 'InternalElement', {
                        'Name': 'Komponenten',
                        'ID': f'rail-{rail_id}-components'
                    })

                    for idx, comp in enumerate(rail_state['components'], 1):
                        self._save_component(comp_elem, idx, rail_id, comp)

            # Datei schreiben
            tree = ET.ElementTree(root)
            ET.indent(tree, space='  ')
            tree.write(state_file_path, encoding='utf-8', xml_declaration=True)
            return True

        except Exception as e:
            print(f"Fehler beim Speichern: {e}")
            return False

    def load_state(self, state_file_path: str) -> Optional[Dict[str, Any]]:
        """Lädt Zustand aus AML-Datei"""
        try:
            if not os.path.exists(state_file_path):
                return None

            tree = ET.parse(state_file_path)
            root = tree.getroot()

            state_data = {
                'workspace_rail_id': -1,
                'rail_states': {},
                'timestamp': None
            }

            # Zustandsinfo laden
            state_info = root.find('.//caex:InternalElement[@Name="Zustandsinfo"]', self.ns)
            if state_info:
                state_data['workspace_rail_id'] = int(
                    self._get_attr_value(state_info, 'Arbeitsplatz-Schiene') or '-1'
                )
                state_data['timestamp'] = self._get_attr_value(state_info, 'Letztes-Update')

            # Schienen laden
            rails_elem = root.find('.//caex:InternalElement[@Name="Hutschienen"]', self.ns)
            if rails_elem:
                for rail_elem in rails_elem.findall('.//caex:InternalElement', self.ns):
                    rail_name = rail_elem.get('Name')
                    if rail_name and rail_name.startswith('Schiene-'):
                        rail_id = int(rail_name.split('-')[1])
                        state_data['rail_states'][rail_id] = self._load_rail_state(rail_elem)

            return state_data

        except Exception as e:
            print(f"Fehler beim Laden: {e}")
            return None

    def _save_component(self, parent: ET.Element, idx: int, rail_id: int, comp: Dict[str, Any]):
        """Speichert eine Komponente"""
        comp_elem = ET.SubElement(parent, 'InternalElement', {
            'Name': f'Position-{idx}',
            'ID': f'rail-{rail_id}-pos-{idx}'
        })

        self._add_attr(comp_elem, 'Komponenten-Typ', 'xs:string', comp['type_name'])
        self._add_attr(comp_elem, 'Original-ID', 'xs:string', comp['object_id'])
        self._add_attr(comp_elem, 'Instance-Number', 'xs:int', str(comp['instance_number']))
        self._add_attr(comp_elem, 'Position-auf-Schiene', 'xs:double',
                       str(comp['position_on_rail']))
        self._add_attr(comp_elem, 'Breite', 'xs:double', str(comp['width']))

    def _load_rail_state(self, rail_elem: ET.Element) -> Dict[str, Any]:
        """Lädt Zustand einer Schiene"""
        rail_state = {
            'location': int(self._get_attr_value(rail_elem, 'Location') or '0'),
            'cabinet_position': int(self._get_attr_value(rail_elem, 'Cabinet-Position') or '-1'),
            'fill_level': float(self._get_attr_value(rail_elem, 'Füllstand') or '0.0'),
            'components': []
        }

        # Komponenten laden
        comp_container = rail_elem.find('.//caex:InternalElement[@Name="Komponenten"]', self.ns)
        if comp_container:
            for comp_elem in comp_container.findall('.//caex:InternalElement', self.ns):
                rail_state['components'].append({
                    'type_name': self._get_attr_value(comp_elem, 'Komponenten-Typ'),
                    'object_id': self._get_attr_value(comp_elem, 'Original-ID'),
                    'instance_number': int(self._get_attr_value(comp_elem, 'Instance-Number') or '0'),
                    'position_on_rail': float(
                        self._get_attr_value(comp_elem, 'Position-auf-Schiene') or '0.0'
                    ),
                    'width': float(self._get_attr_value(comp_elem, 'Breite') or '0.0')
                })

        return rail_state

    def _add_attr(self, parent: ET.Element, name: str, dtype: str, value: str):
        """Fügt XML-Attribut hinzu"""
        attr = ET.SubElement(parent, 'Attribute', {
            'Name': name,
            'AttributeDataType': dtype
        })
        ET.SubElement(attr, 'Value').text = value

    def _get_attr_value(self, element: ET.Element, attr_name: str) -> Optional[str]:
        """Holt Attributwert"""
        for attr in element.findall(f'.//caex:Attribute[@Name="{attr_name}"]', self.ns):
            value = attr.find('.//caex:Value', self.ns)
            if value is not None and value.text:
                return value.text.strip()
        return None


# Test bei direktem Aufruf
if __name__ == "__main__":
    parser = AMLParser()
    if parser.parse():
        print(f"✓ {len(parser.objects)} Objekte geladen")
        print(f"✓ {parser.rail_info.get('count', 0)} Schienen konfiguriert")

        # Zeige Farben und Größenkategorien
        print("\nKomponenten-Details:")
        for name, obj in sorted(parser.objects.items()):
            print(f"- {name}: {obj['color']}, {obj['size_category']}, {obj['width']}mm")

        valid, errors = parser.validate_config()
        if valid:
            print("\n✓ Konfiguration gültig")
        else:
            print("\n✗ Fehler:", ", ".join(errors))