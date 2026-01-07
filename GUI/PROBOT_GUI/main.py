#!/usr/bin/env python3

import sys
import os
import subprocess
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
from pathlib import Path

# Load environment variables from .env file
try:
    from dotenv import load_dotenv
    # Look for .env file in the same directory as this script
    env_path = Path(__file__).parent / '.env'
    load_dotenv(dotenv_path=env_path)
except ImportError:
    print("⚠ python-dotenv not installed. Install with: pip install python-dotenv")
    print("⚠ Falling back to system environment variables only")

# PyQt6 Imports
try:
    from PyQt6.QtWidgets import *
    from PyQt6.QtGui import *
    from PyQt6.QtCore import *
except ImportError as e:
    print(f"✗ Error importing PyQt6: {e}")
    print("\nPlease install PyQt6: pip install PyQt6")
    sys.exit(1)

# Lokale Imports
try:
    from InterActLLM import interactLLM
    from irb120_prompts import (
        generate_level1_prompt,
        generate_level2_prompt,
    )
    from irb120_aml_parser import get_irb120_parser
except ImportError as e:
    print(f"✗ Error importing local modules: {e}")
    print("Please ensure all required files are present")
    sys.exit(1)


# ========== Konfiguration ==========
@dataclass
class Config:
    """Zentrale Konfigurationsklasse"""
    APP_TITLE = "PROBOT - LLM-Aided Robot Programming"
    WINDOW_SIZE = (1200, 900)
    WORKSPACE_PATH = os.path.expanduser("~/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models")
    LOGO_PATH = "Resources/LPS_logo.png"

    # API Keys (must be set as environment variables)
    OPENAI_KEY = os.getenv("OPENAI_API_KEY", "")
    CLAUDE_KEY = os.getenv("CLAUDE_API_KEY", "")

    # ===== NEU: Ollama-Konfiguration =====
    OLLAMA_BASE_URL = os.getenv("OLLAMA_BASE_URL", "http://134.147.216.152:11434")
    OLLAMA_MODEL = "llama3.1:70b"

    # Farben
    COLORS = {
        'level1': "#28a745",
        'level2': "#28a745",
        'computing': "#fd7e14",
        'background': "#1a0a46"
    }

    # Level-Beschreibungen
    LEVEL_DESC = {
        1: "Task Analysis & Planning",
        2: "C++ Code Generation"
    }

    # IRB 120 specific paths
    SCRIPT_PATH = "src/ur10e_hl_interface/src/script.cpp"
    PACKAGE_NAME = "ur10e_hl_interface"
    EXECUTABLE_NAME = "script"


# ========== Basis-Widgets ==========
class StyledDisplay(QTextEdit):
    """Basis-Klasse für gestylte Anzeige-Widgets"""

    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setStyleSheet("""
            QTextEdit {
                background-color: rgba(255, 255, 255, 0.1);
                color: white;
                border: 1px solid #444;
                font-family: 'Courier New', monospace;
                font-size: 12px;
                padding: 5px;
                border-radius: 5px;
            }
        """)


class TipsDisplay(StyledDisplay):
    """Widget zur Anzeige von Tipps und Anleitungen"""

    def __init__(self):
        super().__init__()
        self.setTips()

    def setTips(self):
        """Set the tips text - can be configured later"""
        tips_text = """1. Optional: "Zelle verändern?" aktivieren
   um Würfel hinzuzufügen/entfernen

2. Roboter-Aufgabe beschreiben

3. Weiter drücken für LLM-Analyse

4. Roboter Setup starten & Code ausführen

Beispiele Zellkonfiguration:
- "2 rote Würfel auf A1 und B3"
- "Entferne alle Würfel"

Beispiele Roboter-Aufgabe:
- "Bringe den roten Würfel zur Rampe"
- "Erstelle eine gestapelte Ampel auf C3"
- "Sortiere alle Würfel nach Farbe"
"""
        self.setPlainText(tips_text)


class StackedCubeCell(QWidget):
    """Custom widget to display stacked cubes in a grid cell"""

    # German color names to CSS/Qt colors
    COLOR_MAP = {
        'Schwarz': '#333333',
        'Rot': '#e74c3c',
        'Gruen': '#27ae60',
        'Gelb': '#f1c40f',
        'Blau': '#3498db',
        'Grau': '#95a5a6',
        'Orange': '#e67e22',
    }

    def __init__(self, pos_name: str):
        super().__init__()
        self.pos_name = pos_name
        self.cubes = []  # List of (name, color, level) tuples, sorted by level
        self.setFixedSize(45, 45)

    def setCubes(self, cubes: list):
        """Set the stacked cubes for this cell.

        Args:
            cubes: List of dicts with 'name', 'color', 'stack_level' keys
        """
        self.cubes = sorted(cubes, key=lambda x: x.get('stack_level', 0))
        self._update_tooltip()
        self.update()  # Trigger repaint

    def clearCubes(self):
        """Clear all cubes from this cell"""
        self.cubes = []
        self.setToolTip(f"Position {self.pos_name} - frei")
        self.update()

    def _update_tooltip(self):
        """Update tooltip with cube information"""
        if not self.cubes:
            self.setToolTip(f"Position {self.pos_name} - frei")
        else:
            lines = [f"Position {self.pos_name}:"]
            for cube in self.cubes:
                level = cube.get('stack_level', 0)
                lines.append(f"  Level {level}: {cube.get('name', '?')} ({cube.get('color', '?')})")
            self.setToolTip('\n'.join(lines))

    def paintEvent(self, event):
        """Custom paint event to draw stacked cubes"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        cell_size = 45
        margin = 2
        border_radius = 5

        if not self.cubes:
            # Draw empty cell
            painter.setPen(QPen(QColor('#555555'), 1))
            painter.setBrush(QBrush(QColor(255, 255, 255, 38)))  # rgba(255,255,255,0.15)
            painter.drawRoundedRect(margin, margin, cell_size - 2*margin, cell_size - 2*margin, border_radius, border_radius)

            # Draw position name
            painter.setPen(QColor('#aaaaaa'))
            font = painter.font()
            font.setPointSize(9)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self.pos_name)
        else:
            # Draw stacked cubes
            # Each level shrinks by a fixed amount from bottom-right
            # Level 0: full size, Level 1: smaller, Level 2: even smaller, etc.
            shrink_per_level = 6  # pixels to shrink per level
            max_levels = 4

            for cube in self.cubes:
                level = min(cube.get('stack_level', 0), max_levels - 1)
                color_name = cube.get('color', 'Grau')
                css_color = self.COLOR_MAP.get(color_name, '#666666')

                # Calculate rectangle: upper-left fixed, lower-right shrinks
                shrink = level * shrink_per_level
                x = margin
                y = margin
                w = cell_size - 2*margin - shrink
                h = cell_size - 2*margin - shrink

                # Draw cube rectangle
                painter.setPen(QPen(QColor('#ffffff'), 2))
                painter.setBrush(QBrush(QColor(css_color)))
                painter.drawRoundedRect(x, y, w, h, border_radius, border_radius)

            # Draw stack count if multiple cubes (outside the cubes, bottom-right)
            if len(self.cubes) > 1:
                # Draw a small background circle for the count
                count_text = str(len(self.cubes))
                painter.setPen(QPen(QColor('#ffffff'), 1))
                painter.setBrush(QBrush(QColor('#333333')))
                # Position in bottom-right corner of cell, outside the smallest cube
                circle_x = cell_size - 12
                circle_y = cell_size - 12
                painter.drawEllipse(circle_x, circle_y, 10, 10)

                # Draw the number
                painter.setPen(QColor('#ffffff'))
                font = painter.font()
                font.setPointSize(7)
                font.setBold(True)
                painter.setFont(font)
                painter.drawText(circle_x, circle_y, 10, 10,
                               Qt.AlignmentFlag.AlignCenter, count_text)


class CellConfigDisplay(QWidget):
    """Widget zur Anzeige der 5x5 Zellkonfiguration mit farbigen Würfeln"""

    def __init__(self):
        super().__init__()
        self.grid_cells = {}  # Position name -> StackedCubeCell
        self._setup_grid()

    def _setup_grid(self):
        """Create the 5x5 grid layout"""
        layout = QGridLayout(self)
        layout.setSpacing(4)
        layout.setContentsMargins(5, 5, 5, 5)

        # Create 5x5 grid (A-E columns, 1-5 rows)
        # Row 5 is at top (closest to robot), Row 1 at bottom
        columns = ['A', 'B', 'C', 'D', 'E']
        rows = ['5', '4', '3', '2', '1']  # Reversed for visual layout

        for row_idx, row_num in enumerate(rows):
            for col_idx, col_letter in enumerate(columns):
                pos_name = f"{col_letter}{row_num}"
                cell = StackedCubeCell(pos_name)
                self.grid_cells[pos_name] = cell
                layout.addWidget(cell, row_idx, col_idx)

    def updateGrid(self):
        """Update grid with current cube positions from AML"""
        try:
            parser = get_irb120_parser()
            parser.reload()

            # Reset all cells to empty
            for pos_name, cell in self.grid_cells.items():
                cell.clearCubes()

            # Group objects by base position
            position_cubes = {}  # base_position -> list of cube dicts
            for obj in parser.get_current_objects():
                base_pos = obj.get('base_position', obj.get('location', ''))
                if base_pos and base_pos in self.grid_cells:
                    if base_pos not in position_cubes:
                        position_cubes[base_pos] = []
                    position_cubes[base_pos].append(obj)

            # Update cells with stacked cubes
            for pos, cubes in position_cubes.items():
                self.grid_cells[pos].setCubes(cubes)

        except Exception as e:
            print(f"Fehler beim Aktualisieren des Rasters: {e}")


class HoverButton(QPushButton):
    """Button mit Hover-Effekt"""

    def __init__(self, text: str, color: str, hover_color: str):
        super().__init__(text)
        self.default_color = color
        self.hover_color = hover_color
        self._apply_style(color)

    def _apply_style(self, color: str):
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                padding: 8px;
                border-radius: 5px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {self.hover_color};
            }}
        """)

    def setColor(self, color: str):
        self.default_color = color
        self._apply_style(color)


# ========== Thread-Klassen ==========
class WorkerThread(QThread):
    """Basis-Klasse für Worker-Threads"""
    output = pyqtSignal(str)
    error = pyqtSignal(str)
    finished = pyqtSignal(bool)

    def __init__(self, workspace_path: str):
        super().__init__()
        self.workspace_path = workspace_path
        self._is_running = True

    def stop(self):
        """Stoppt den Thread sauber"""
        self._is_running = False


class SetupRobotThread(WorkerThread):
    """Thread zum Starten der Roboter-Umgebung"""

    def run(self):
        try:
            self.output.emit("Starte Roboter-Umgebung...")
            launcher_path = os.path.join(self.workspace_path, "launcher.py")

            if not os.path.exists(launcher_path):
                self.error.emit(f"launcher.py nicht gefunden unter: {launcher_path}")
                self.finished.emit(False)
                return

            cmd = f"cd {self.workspace_path} && python3 launcher.py"
            subprocess.Popen(
                f"gnome-terminal --title='Robot Setup' -- bash -c '{cmd}'",
                shell=True
            )
            self.finished.emit(True)

        except Exception as e:
            self.error.emit(f"Setup-Fehler: {str(e)}")
            self.finished.emit(False)


class ExecuteCodeThread(WorkerThread):
    """Thread zum Kompilieren und Ausführen des generierten Codes"""

    def __init__(self, workspace_path: str, package_name: str = "ur10e_hl_interface",
                 executable_name: str = "script", real_robot: bool = False):
        super().__init__(workspace_path)
        self.package_name = package_name
        self.executable_name = executable_name
        self.real_robot = real_robot

    def run(self):
        try:
            # Build real_robot argument for launch file
            real_robot_arg = "real_robot:=true" if self.real_robot else "real_robot:=false"
            mode_text = "REAL ROBOT" if self.real_robot else "SIMULATION"

            # Use ros2 launch to properly pass MoveIt parameters to the script
            build_cmd = (
                f'cd {self.workspace_path} && '
                f'source /opt/ros/humble/setup.bash && '
                f'source install/setup.bash && '
                f'echo "=== Kompiliere {self.package_name} ===" && '
                f'colcon build --packages-select {self.package_name} && '
                f'source install/setup.bash && '
                f'echo && echo "=== Build abgeschlossen ===" && echo && '
                f'echo "Mode: {mode_text}" && echo && '
                f'read -p "Build erfolgreich? Code ausfuehren? [j/n]: " response && '
                f'if [ "$response" = "y" ] || [ "$response" = "j" ]; then '
                f'  echo && echo "=== Starte Ausfuehrung ({mode_text}) ===" && '
                f'  ros2 launch {self.package_name} execute_script.launch.py {real_robot_arg}; '
                f'else '
                f'  echo "Ausfuehrung abgebrochen."; '
                f'fi; '
                f'echo && echo "Prozess beendet. ENTER zum Schliessen..."; read'
            )

            subprocess.Popen(
                f"gnome-terminal --tab --title='Build & Ausfuehrung' -- bash -c '{build_cmd}'",
                shell=True
            )

            self.output.emit(f"Build und Ausführung gestartet für {self.executable_name}.")
            self.finished.emit(True)

        except Exception as e:
            self.error.emit(f"Ausführungsfehler: {str(e)}")
            self.finished.emit(False)


class LLMGenerationThread(QThread):
    """Thread für LLM-Generierung um GUI-Blockierung zu vermeiden"""
    output = pyqtSignal(str)  # Für Status-Updates
    response_ready = pyqtSignal(str, int)  # (response_text, level)
    error = pyqtSignal(str)

    def __init__(self, api_key: str, model: str, full_prompt: str, level: int,
                 ollama_base_url: str = None):
        super().__init__()
        self.api_key = api_key
        self.model = model
        self.full_prompt = full_prompt
        self.level = level
        self.ollama_base_url = ollama_base_url

    def run(self):
        try:
            # Wähle API basierend auf Modell
            if self.model.startswith("claude"):
                # Anthropic Cloud
                client = interactLLM(
                    api_key=self.api_key,
                    model=self.model,
                    api_type="anthropic"
                )
                self.output.emit(f"Verwende Claude: {self.model}")

            elif self.model.startswith("ollama:"):
                # Ollama (Netzwerk-Server)
                actual_model = self.model.replace("ollama:", "")
                client = interactLLM(
                    api_key=None,
                    model=actual_model,
                    api_type="ollama",
                    base_url=self.ollama_base_url
                )
                self.output.emit(f"Verwende Llama (Netzwerk): {actual_model}")

            else:
                # OpenAI Cloud
                client = interactLLM(
                    api_key=self.api_key,
                    model=self.model,
                    api_type="openai"
                )
                self.output.emit(f"Verwende OpenAI: {self.model}")

            # Generiere Antwort (blockierend, aber in separatem Thread)
            response = client.generate_response(
                prompt=self.full_prompt,
                max_tokens=10000,
                temperature=0.3  # Niedrig für deterministische Ausgaben
            )

            # Sende Antwort zurück zur GUI
            self.response_ready.emit(response, self.level)

        except Exception as e:
            self.error.emit(f"Fehler bei der Generierung: {str(e)}")


# ========== Haupt-GUI ==========
class ProbotGUI(QMainWindow):
    """
    Hauptfenster der PROBOT-Anwendung

    Verwaltet die Benutzeroberfläche für die LLM-gestützte
    Roboterprogrammierung mit zwei Verarbeitungsebenen.
    """

    def __init__(self):
        super().__init__()
        self.config = Config()
        self._init_state()
        self._setup_ui()
        self._connect_signals()
        self._start_refresh_timer()

    def _init_state(self):
        """Initialisiert den Anwendungszustand"""
        self.current_level = 1
        self.current_cell_setup = ""  # Zellkonfigurations-Beschreibung
        self.current_robot_task = ""  # Roboter-Aufgaben-Beschreibung
        self.responses = {1: "", 2: ""}
        self.generated_code = ""
        self.robot_ready = False
        self.total_placed = 0

        # Thread-Referenzen initialisieren
        self.setup_thread = None
        self.execute_thread = None
        self.llm_thread = None  # NEU: LLM-Thread

    def _setup_ui(self):
        """Erstellt die Benutzeroberfläche"""
        self.setWindowTitle(self.config.APP_TITLE)
        self.setGeometry(100, 100, *self.config.WINDOW_SIZE)

        # Zentrales Widget
        central = QWidget()
        self.setCentralWidget(central)

        # Hauptlayout
        layout = QVBoxLayout(central)
        layout.setSpacing(10)
        layout.setContentsMargins(20, 20, 20, 20)

        # UI-Komponenten erstellen
        self._create_header(layout)
        self._create_status_bar(layout)
        self._create_main_area(layout)
        self._create_input_area(layout)
        self._create_options(layout)
        self._create_controls(layout)
        self._create_progress_bar(layout)

        # Hintergrundfarbe setzen
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {self.config.COLORS['background']};
            }}
        """)

    def _create_header(self, layout: QVBoxLayout):
        """Erstellt den Header-Bereich"""
        header = QHBoxLayout()

        # Titel
        title_layout = QVBoxLayout()
        title = QLabel("PROBOT")
        title.setFont(QFont("Arial", 24, QFont.Weight.Bold))
        title.setStyleSheet("color: white;")

        subtitle = QLabel("LLM-Aided Robot Programming")
        subtitle.setFont(QFont("Arial", 14))
        subtitle.setStyleSheet("color: lightgray;")

        title_layout.addWidget(title)
        title_layout.addWidget(subtitle)

        # Logo
        logo = QLabel()
        if os.path.exists(self.config.LOGO_PATH):
            pixmap = QPixmap(self.config.LOGO_PATH).scaled(
                100, 100, Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            logo.setPixmap(pixmap)

        header.addLayout(title_layout)
        header.addStretch()
        header.addWidget(logo)
        layout.addLayout(header)

    def _create_status_bar(self, layout: QVBoxLayout):
        """Erstellt die Statusanzeige"""
        self.status_label = QLabel(self._get_status_text())
        self.status_label.setFixedHeight(30)
        self._update_status_style()
        layout.addWidget(self.status_label)

    def _create_main_area(self, layout: QVBoxLayout):
        """Erstellt den Hauptbereich mit Ausgabe- und Info-Anzeigen"""
        main_layout = QHBoxLayout()

        # Left: LLM Output
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setPlaceholderText("LLM-generierte Ausgaben")
        self.output_text.setStyleSheet("""
            QTextEdit {
                background-color: white;
                color: black;
                border-radius: 5px;
                font-family: Arial;
                font-size: 12px;
            }
        """)

        # Right: Info Displays
        info_layout = QVBoxLayout()

        # Tips / Instructions
        tips_group = self._create_group_box("Tipps")
        self.tips_display = TipsDisplay()
        tips_group.layout().addWidget(self.tips_display)

        # Cell Configuration (5x5 Grid)
        cell_group = self._create_group_box("Zellkonfiguration")
        self.cell_config_display = CellConfigDisplay()
        cell_group.layout().addWidget(self.cell_config_display)

        info_layout.addWidget(tips_group, 1)
        info_layout.addWidget(cell_group, 1)

        main_layout.addWidget(self.output_text, 2)
        main_layout.addLayout(info_layout, 1)
        layout.addLayout(main_layout)

    def _create_group_box(self, title: str) -> QGroupBox:
        """Erstellt eine gestylte GroupBox"""
        group = QGroupBox(title)
        group.setLayout(QVBoxLayout())
        group.setStyleSheet("""
            QGroupBox {
                color: white;
                border: 1px solid #444;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        return group

    def _create_input_area(self, layout: QVBoxLayout):
        """Erstellt den Eingabebereich mit Checkbox für Zellveränderung"""
        input_container = QVBoxLayout()

        # Checkbox für Zellveränderung
        self.cell_change_checkbox = QCheckBox("Zell-Setup verändern?")
        self.cell_change_checkbox.setStyleSheet("""
            QCheckBox {
                color: white;
                font-weight: bold;
                spacing: 8px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                background-color: rgba(255, 255, 255, 0.2);
                border: 2px solid #666;
                border-radius: 4px;
            }
            QCheckBox::indicator:checked {
                background-color: #28a745;
                border: 2px solid #28a745;
            }
        """)
        self.cell_change_checkbox.stateChanged.connect(self._toggle_cell_input)
        input_container.addWidget(self.cell_change_checkbox)

        # Eingabefeld für Zellkonfiguration (initial versteckt)
        self.cell_input_container = QWidget()
        cell_layout = QVBoxLayout(self.cell_input_container)
        cell_layout.setContentsMargins(0, 5, 0, 5)

        cell_label = QLabel("Zellkonfiguration beschreiben:")
        cell_label.setStyleSheet("color: #aaaaaa; font-size: 11px;")
        cell_layout.addWidget(cell_label)

        self.cell_input_text = QTextEdit()
        self.cell_input_text.setPlaceholderText(
            "z.B. \"2 rote Würfel auf A1 und B3\" oder \"Entferne alle Würfel\""
        )
        self.cell_input_text.setFixedHeight(60)
        self.cell_input_text.setStyleSheet("""
            QTextEdit {
                background-color: white;
                color: black;
                border-radius: 5px;
                font-family: Arial;
                font-size: 12px;
            }
        """)
        cell_layout.addWidget(self.cell_input_text)

        self.cell_input_container.setVisible(False)
        input_container.addWidget(self.cell_input_container)

        # Eingabefeld für Roboter-Aufgabe
        task_label = QLabel("Roboter-Aufgabe beschreiben:")
        task_label.setStyleSheet("color: #aaaaaa; font-size: 11px;")
        input_container.addWidget(task_label)

        self.input_text = QTextEdit()
        self.input_text.setPlaceholderText(
            "z.B. \"Bringe den roten Würfel zur Rampe\" oder \"Verschiebe Würfel von A1 nach B2\""
        )
        self.input_text.setFixedHeight(70)
        self.input_text.setStyleSheet("""
            QTextEdit {
                background-color: white;
                color: black;
                border-radius: 5px;
                font-family: Arial;
                font-size: 12px;
            }
        """)
        input_container.addWidget(self.input_text)

        layout.addLayout(input_container)

    def _toggle_cell_input(self, state):
        """Zeigt/versteckt das Zellkonfigurations-Eingabefeld"""
        self.cell_input_container.setVisible(state == Qt.CheckState.Checked.value)

    def _create_options(self, layout: QVBoxLayout):
        """Erstellt die Level-1-Optionen"""
        self.options_group = QGroupBox("Optionen nach Analyse")
        self.options_group.setStyleSheet("""
            QGroupBox {
                color: white;
                background-color: rgba(255, 255, 255, 0.1);
                border-radius: 5px;
                padding: 8px;
            }
        """)

        options_layout = QHBoxLayout()
        self.radio_proceed = QRadioButton("Zur Code-Generierung")
        self.radio_proceed.setChecked(True)
        self.radio_regenerate = QRadioButton("Analyse wiederholen")
        self.radio_edit = QRadioButton("Prompt bearbeiten")

        # Verbessertes Styling für Radio-Buttons mit grünem Indikator für Auswahl
        radio_style = """
            QRadioButton {
                color: white;
                spacing: 5px;
            }
            QRadioButton::indicator {
                width: 18px;
                height: 18px;
                background-color: rgba(255, 255, 255, 0.2);
                border: 2px solid #666;
                border-radius: 9px;
            }
            QRadioButton::indicator:checked {
                background-color: #28a745;
                border: 2px solid #28a745;
            }
            QRadioButton::indicator:unchecked:hover {
                border: 2px solid #aaa;
            }
        """

        for radio in [self.radio_proceed, self.radio_regenerate, self.radio_edit]:
            radio.setStyleSheet(radio_style)
            options_layout.addWidget(radio)

        options_layout.addStretch()
        self.options_group.setLayout(options_layout)
        self.options_group.setVisible(False)
        self.options_group.setMaximumHeight(60)
        layout.addWidget(self.options_group)

    def _create_controls(self, layout: QVBoxLayout):
        """Erstellt die Steuerelemente"""
        controls = QHBoxLayout()

        # Buttons
        self.btn_proceed = HoverButton("Weiter", "#28a745", "#218838")
        self.btn_setup = HoverButton("Roboter starten", "#007bff", "#0056b3")
        self.btn_execute = HoverButton("Code ausführen", "#17a2b8", "#138496")
        self.btn_reset = HoverButton("Zelle zurücksetzen", "#ffc107", "#e0a800")
        self.btn_close = HoverButton("Beenden", "#DC3545", "#a71d2a")

        # Real Robot checkbox
        self.real_robot_checkbox = QCheckBox("Real Robot")
        self.real_robot_checkbox.setToolTip(
            "Wenn aktiviert, werden Befehle auch an den echten Roboter gesendet.\n"
            "Stellen Sie sicher, dass SocketMain auf dem FlexPendant läuft!"
        )
        self.real_robot_checkbox.setStyleSheet("""
            QCheckBox {
                color: white;
                font-weight: bold;
                spacing: 8px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                background-color: rgba(255, 255, 255, 0.2);
                border: 2px solid #666;
                border-radius: 4px;
            }
            QCheckBox::indicator:checked {
                background-color: #dc3545;
                border: 2px solid #dc3545;
            }
            QCheckBox::indicator:unchecked:hover {
                border: 2px solid #aaa;
            }
        """)

        # Model-Auswahl
        self.model_select = QComboBox()
        self.model_select.addItems([
            # Claude Models (Recommended)
            "claude-sonnet-4-5",         # Latest Sonnet 4.5 (DEFAULT)
            "claude-opus-4-5",           # Most capable
            "claude-sonnet-4",           # Sonnet 4
            "claude-3-5-sonnet",         # 3.5 Sonnet (fallback)
            "claude-3-opus",             # 3 Opus
            # OpenAI Models
            "gpt-4o",
            "gpt-4",
            # Local Models (Network Server)
            "ollama:llama3.1:70b",
        ])
        self.model_select.setCurrentIndex(0)  # Claude Sonnet 4.5 as default
        self.model_select.setStyleSheet("""
            QComboBox {
                background-color: white;
                color: black;
                padding: 5px;
                border-radius: 5px;
            }
        """)

        # Zur Layout hinzufügen
        for widget in [self.btn_proceed, self.btn_setup, self.btn_execute,
                       self.real_robot_checkbox, self.btn_reset,
                       self.model_select, self.btn_close]:
            controls.addWidget(widget)

        layout.addLayout(controls)

    def _create_progress_bar(self, layout: QVBoxLayout):
        """Erstellt die Fortschrittsanzeige"""
        self.progress = QProgressBar()
        self.progress.setVisible(False)
        self.progress.setStyleSheet("""
            QProgressBar {
                background-color: white;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #17a2b8;
                border-radius: 5px;
            }
        """)
        layout.addWidget(self.progress)

    def _connect_signals(self):
        """Verbindet Signale mit Slots"""
        self.btn_proceed.clicked.connect(self.handle_proceed)
        self.btn_setup.clicked.connect(self.handle_setup)
        self.btn_execute.clicked.connect(self.handle_execute)
        self.btn_reset.clicked.connect(self.handle_reset)
        self.btn_close.clicked.connect(self.close)

    def _start_refresh_timer(self):
        """Startet Timer für automatische Aktualisierungen"""
        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh_displays)
        self.timer.start(5000)  # Alle 5 Sekunden

        # Initiale Aktualisierung
        self._refresh_displays()

    def _refresh_displays(self):
        """Aktualisiert die Zellkonfiguration mit aktuellem AML-Stand"""
        try:
            self.cell_config_display.updateGrid()
        except Exception as e:
            print(f"Fehler beim Aktualisieren der Anzeige: {e}")

    # ========== Hilfsmethoden ==========
    def _get_status_text(self) -> str:
        """Gibt den aktuellen Statustext zurück"""
        return f"Status: Level {self.current_level} - {self.config.LEVEL_DESC[self.current_level]}"

    def _update_status_style(self):
        """Aktualisiert das Styling der Statusanzeige"""
        color = self.config.COLORS[f'level{self.current_level}']
        self.status_label.setStyleSheet(f"""
            QLabel {{
                color: white;
                background-color: {color};
                padding: 3px 10px;
                border-radius: 3px;
                font-family: Arial;
            }}
        """)

    def _show_computing(self, computing: bool = True):
        """Zeigt oder versteckt den Berechnungsstatus"""
        if computing:
            self.status_label.setText(f"{self._get_status_text()} - Berechne...")
            self.status_label.setStyleSheet(f"""
                QLabel {{
                    color: white;
                    background-color: {self.config.COLORS['computing']};
                    padding: 5px;
                    border-radius: 3px;
                }}
            """)
        else:
            self.status_label.setText(self._get_status_text())
            self._update_status_style()
        QApplication.processEvents()

    def show_output(self, message: str, msg_type: str = "info"):
        """
        Zeigt eine Nachricht im Output-Bereich an

        Args:
            message: Die anzuzeigende Nachricht
            msg_type: Typ der Nachricht (user, ai, error, system, info)
        """
        cursor = self.output_text.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)

        # Format basierend auf Typ
        format_map = {
            'user': ("You: ", "black", QFont.Weight.Bold),
            'ai': ("PROBOT: ", "blue", QFont.Weight.Bold),
            'error': ("", "red", QFont.Weight.Bold),
            'system': ("", "green", QFont.Weight.Normal),
            'info': ("", "black", QFont.Weight.Normal)
        }

        prefix, color, weight = format_map.get(msg_type, ("", "black", QFont.Weight.Normal))

        # Prefix einfügen
        if prefix:
            bold_format = QTextCharFormat()
            bold_format.setFontWeight(weight)
            bold_format.setForeground(QColor(color))
            cursor.setCharFormat(bold_format)
            cursor.insertText(prefix)

        # Nachricht einfügen
        normal_format = QTextCharFormat()
        normal_format.setFontWeight(QFont.Weight.Normal)
        normal_format.setForeground(QColor(color if not prefix else "black"))
        cursor.setCharFormat(normal_format)
        cursor.insertText(f"{message}\n")

        self.output_text.setTextCursor(cursor)
        self.output_text.ensureCursorVisible()

    # ========== Event Handler ==========
    def handle_proceed(self):
        """Verarbeitet den Weiter-Button"""
        if self.options_group.isVisible():
            # Level 1 Optionen sichtbar
            if self.radio_proceed.isChecked():
                self.current_level = 2
                self.status_label.setText(self._get_status_text())
                self._update_status_style()
                self.options_group.setVisible(False)
                self.generate_response(2)
            elif self.radio_regenerate.isChecked():
                self.options_group.setVisible(False)
                self.generate_response(1)
            elif self.radio_edit.isChecked():
                # Stelle vorherige Eingaben wieder her
                self.input_text.setPlainText(self.current_robot_task)
                if self.current_cell_setup:
                    self.cell_change_checkbox.setChecked(True)
                    self.cell_input_text.setPlainText(self.current_cell_setup)
                self.options_group.setVisible(False)
        elif self.current_level == 1:
            # Erste Generierung von Level 1
            self.generate_response(1)
        else:
            # Nach Level 2 zurück zu Level 1
            self.reset_to_level1()

    def handle_setup(self):
        """Startet das Robot-Setup"""
        if self.robot_ready:
            self.robot_ready = False
            self.btn_setup.setText("Roboter starten")
            self.btn_setup.setColor("#007bff")
            return

        # Thread als Instanzvariable speichern, damit er nicht vorzeitig zerstört wird
        self.setup_thread = SetupRobotThread(self.config.WORKSPACE_PATH)
        self.setup_thread.finished.connect(lambda success: self._on_setup_finished(success))
        self.setup_thread.start()

    def handle_execute(self):
        """Executes the generated code"""
        real_robot = self.real_robot_checkbox.isChecked()

        # Show confirmation dialog for real robot mode
        if real_robot:
            reply = QMessageBox.warning(
                self, 'Real Robot Mode',
                '⚠️ WARNUNG: Der ECHTE ROBOTER wird bewegt!\n\n'
                'Stellen Sie sicher, dass:\n'
                '• SocketMain auf dem FlexPendant läuft\n'
                '• Der Roboter im AUTO-Modus mit Motors On ist\n'
                '• Der Arbeitsbereich frei ist\n\n'
                'Fortfahren?',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No
            )
            if reply != QMessageBox.StandardButton.Yes:
                self.show_output("Real Robot Ausführung abgebrochen.", "system")
                return

        # Create thread with IRB 120 specific parameters
        self.execute_thread = ExecuteCodeThread(
            self.config.WORKSPACE_PATH,
            self.config.PACKAGE_NAME,
            self.config.EXECUTABLE_NAME,
            real_robot=real_robot
        )
        self.execute_thread.output.connect(lambda msg: self.show_output(msg, "system"))
        self.execute_thread.error.connect(lambda msg: self.show_output(msg, "error"))
        self.execute_thread.finished.connect(lambda success: QTimer.singleShot(3000, self._refresh_displays))
        self.execute_thread.start()

    def handle_reset(self):
        """Setzt die Zelle auf Standardkonfiguration zurück"""
        reply = QMessageBox.question(
            self, 'Zelle zurücksetzen',
            'Zelle auf Standardkonfiguration zurücksetzen?\n\n'
            'Dies erstellt 4 Würfel:\n'
            '- Schwarz auf A1\n'
            '- Grün auf B1\n'
            '- Rot auf C1\n'
            '- Gelb auf D1',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            try:
                self._reset_cell_to_default()
                self._refresh_displays()
                self.show_output("Zelle auf Standardkonfiguration zurückgesetzt!", "system")
            except Exception as e:
                self.show_output(f"Fehler beim Zurücksetzen: {str(e)}", "error")

    def _reset_cell_to_default(self):
        """Schreibt die Standard-Zellkonfiguration in die AML-Datei"""
        aml_content = '''<?xml version="1.0" encoding="utf-8"?>
<CAEXFile xmlns="http://www.dke.de/CAEX" FileName="irb120_simple_config.aml" SchemaVersion="3.0">
  <InstanceHierarchy Name="Robot-Config">
    <InternalElement Name="IRB120">
      <Attribute Name="ApproachHeight">
        <Value>0.05</Value>
      </Attribute>
      <Attribute Name="VelocityScaling">
        <Value>0.3</Value>
      </Attribute>
      <Attribute Name="AccelerationScaling">
        <Value>0.3</Value>
      </Attribute>
      <Attribute Name="PlannerID">
        <Value>RRTConnect</Value>
      </Attribute>
      <Attribute Name="GripperOrientation">
        <Value>0.0,0.7071068,0.7071068,0.0</Value>
      </Attribute>
    </InternalElement>
  </InstanceHierarchy>
  <InstanceHierarchy Name="Grid-Config">
    <InternalElement Name="GridParameters">
      <Attribute Name="A1SpawnPosition">
        <Value>0.495,-0.090,0.0</Value>
      </Attribute>
      <Attribute Name="GridSpacing">
        <Value>0.0475</Value>
      </Attribute>
      <Attribute Name="GripOffset">
        <Value>0.015,0.015,0.1</Value>
      </Attribute>
    </InternalElement>
    <InternalElement Name="SpecialPositions">
      <InternalElement Name="X">
        <Attribute Name="Position"><Value>0.550,0.000,0.35</Value></Attribute>
      </InternalElement>
    </InternalElement>
  </InstanceHierarchy>
  <InstanceHierarchy Name="Objects">
    <InternalElement Name="wuerfel_schwarz">
      <Attribute Name="Location"><Value>A1</Value></Attribute>
      <Attribute Name="Color"><Value>Schwarz</Value></Attribute>
      <Attribute Name="Dimensions"><Value>0.03,0.03,0.03</Value></Attribute>
    </InternalElement>
    <InternalElement Name="wuerfel_gruen">
      <Attribute Name="Location"><Value>B1</Value></Attribute>
      <Attribute Name="Color"><Value>Gruen</Value></Attribute>
      <Attribute Name="Dimensions"><Value>0.03,0.03,0.03</Value></Attribute>
    </InternalElement>
    <InternalElement Name="wuerfel_rot">
      <Attribute Name="Location"><Value>C1</Value></Attribute>
      <Attribute Name="Color"><Value>Rot</Value></Attribute>
      <Attribute Name="Dimensions"><Value>0.03,0.03,0.03</Value></Attribute>
    </InternalElement>
    <InternalElement Name="wuerfel_gelb">
      <Attribute Name="Location"><Value>D1</Value></Attribute>
      <Attribute Name="Color"><Value>Gelb</Value></Attribute>
      <Attribute Name="Dimensions"><Value>0.03,0.03,0.03</Value></Attribute>
    </InternalElement>
  </InstanceHierarchy>
</CAEXFile>
'''
        # Schreibe AML-Datei
        aml_path = os.path.join(
            self.config.WORKSPACE_PATH,
            "src/ur10e_hl_interface/config/irb120_simple_config.aml"
        )
        with open(aml_path, 'w', encoding='utf-8') as f:
            f.write(aml_content)

        # Aktualisiere Parser-Cache
        parser = get_irb120_parser()
        parser.reload()

    def generate_response(self, level: int):
        """Generiert eine LLM-Antwort für das angegebene Level (non-blocking)"""
        # Validierung für Level 1
        if level == 1:
            robot_task = self.input_text.toPlainText().strip()
            cell_setup = ""

            # Zellkonfiguration nur wenn Checkbox aktiviert
            if self.cell_change_checkbox.isChecked():
                cell_setup = self.cell_input_text.toPlainText().strip()

            # Mindestens eine Eingabe erforderlich
            if not robot_task and not cell_setup:
                self.show_output("Bitte geben Sie mindestens eine Aufgabenbeschreibung ein.", "error")
                return

            self.current_cell_setup = cell_setup
            self.current_robot_task = robot_task

        self.show_output(f"Generiere Level {level} Antwort...", "user")
        self._show_computing(True)

        # Hole aktuelle System-Prompts mit den beiden Eingaben
        if level == 1:
            system_prompt = generate_level1_prompt(
                cell_setup_text=self.current_cell_setup,
                robot_task_text=self.current_robot_task
            )
        else:
            system_prompt = generate_level2_prompt()

        # Erstelle vollständigen Prompt
        full_prompt = system_prompt

        if level == 2:
            full_prompt += f"\n\nProcess Analysis from Level 1:\n{self.responses[1]}"

        # Hole API-Key basierend auf Modell
        model = self.model_select.currentText()
        if model.startswith("claude"):
            api_key = self.config.CLAUDE_KEY
        elif model.startswith("ollama:"):
            api_key = None  # Ollama braucht keinen API-Key
        else:
            api_key = self.config.OPENAI_KEY

        # Starte LLM-Thread (non-blocking!)
        self.llm_thread = LLMGenerationThread(
            api_key=api_key,
            model=model,
            full_prompt=full_prompt,
            level=level,
            ollama_base_url=self.config.OLLAMA_BASE_URL
        )
        self.llm_thread.output.connect(lambda msg: self.show_output(msg, "system"))
        self.llm_thread.response_ready.connect(self._handle_llm_response)
        self.llm_thread.error.connect(self._handle_llm_error)
        self.llm_thread.start()

    def _handle_llm_response(self, response: str, level: int):
        """Callback wenn LLM-Antwort fertig ist"""
        # Speichere und zeige Antwort
        self.responses[level] = response
        level_name = self.config.LEVEL_DESC[level]
        self.show_output(f"Level {level} - {level_name}:\n{response}", "ai")

        # Level-spezifische Aktionen
        if level == 1:
            self.options_group.setVisible(True)
        elif level == 2:
            self.generated_code = response
            self._save_generated_code()
            self.btn_execute.setEnabled(True)
            self.btn_proceed.setText("Neu starten")

        self._show_computing(False)

    def _handle_llm_error(self, error_msg: str):
        """Callback wenn LLM-Fehler auftritt"""
        self._show_computing(False)
        self.show_output(error_msg, "error")

    def reset_to_level1(self):
        """Setzt die Anwendung auf Level 1 zurück"""
        self.current_level = 1
        self.current_cell_setup = ""
        self.current_robot_task = ""
        self.responses = {1: "", 2: ""}
        self.generated_code = ""
        self.output_text.clear()
        self.input_text.clear()
        self.cell_input_text.clear()
        self.cell_change_checkbox.setChecked(False)
        self.status_label.setText(self._get_status_text())
        self._update_status_style()
        self.options_group.setVisible(False)
        self.btn_execute.setEnabled(False)
        self.btn_proceed.setText("Weiter")
        self._refresh_displays()

    def _save_generated_code(self):
        """Speichert den generierten Code in script.cpp"""
        try:
            # Remove markdown formatting if present
            code = self.generated_code.strip()
            if code.startswith("```"):
                lines = code.split('\n')
                # Find the closing ``` and remove both opening and closing
                if len(lines) > 2:
                    # Skip first line (```cpp or ```) and last line (```)
                    end_idx = len(lines) - 1
                    for i in range(len(lines) - 1, 0, -1):
                        if lines[i].strip() == "```":
                            end_idx = i
                            break
                    code = '\n'.join(lines[1:end_idx])

            file_path = os.path.join(
                self.config.WORKSPACE_PATH,
                self.config.SCRIPT_PATH
            )

            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'w') as f:
                f.write(code)

            self.show_output(f"Code gespeichert unter: {file_path}", "system")

        except Exception as e:
            self.show_output(f"Fehler beim Speichern: {str(e)}", "error")

    def _on_setup_finished(self, success: bool):
        """Callback nach Robot-Setup"""
        if success:
            self.robot_ready = True
            self.btn_setup.setText("Roboter bereit ✓")
            self.btn_setup.setColor("#28a745")
            self.show_output("Roboter-Setup abgeschlossen! Bereit zur Code-Ausführung.", "system")
        else:
            self.btn_setup.setText("Roboter starten")
            self.btn_setup.setColor("#007bff")

    def closeEvent(self, event):
        """Behandelt das Schließen des Fensters"""
        # Timer stoppen
        self.timer.stop()

        # Threads ordnungsgemäß beenden
        for thread in [self.setup_thread, self.execute_thread, self.llm_thread]:
            if thread and thread.isRunning():
                thread.stop() if hasattr(thread, 'stop') else None  # Signal zum Stoppen senden
                thread.quit()
                if not thread.wait(1000):  # Maximal 1 Sekunde warten
                    thread.terminate()  # Notfalls terminieren

        event.accept()


# ========== Hauptfunktion ==========
def main():
    """Haupteinstiegspunkt der Anwendung"""
    try:
        app = QApplication(sys.argv)

        # Setze Anwendungs-Stil
        app.setStyle('Fusion')

        # Erstelle und zeige Hauptfenster
        window = ProbotGUI()
        window.show()

        sys.exit(app.exec())

    except Exception as e:
        print(f"Critical error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()