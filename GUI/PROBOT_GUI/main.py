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
    from prompts import (
        get_available_quantities,
        get_cabinet_state,
        generate_level1_prompt,
        generate_level2_prompt
    )
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
        1: "Process Analysis & Structuring",
        2: "Code Generation"
    }


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


class ComponentsDisplay(StyledDisplay):
    def __init__(self):
        super().__init__()
        self._last_text = ""

    def setComponents(self, text: str):
        if text != self._last_text:
            self._last_text = text
            self.setPlainText(text)


class CabinetStateDisplay(StyledDisplay):
    """Widget zur Anzeige des Schaltschrank-Zustands"""

    def updateState(self):
        try:
            self.setPlainText(get_cabinet_state())
        except Exception as e:
            self.setPlainText(f"Error loading cabinet state: {str(e)}")


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
            self.output.emit("Starting robot environment setup...")
            launcher_path = os.path.join(self.workspace_path, "launcher.py")

            if not os.path.exists(launcher_path):
                self.error.emit(f"launcher.py not found at: {launcher_path}")
                self.finished.emit(False)
                return

            cmd = f"cd {self.workspace_path} && python3 launcher.py"
            subprocess.Popen(
                f"gnome-terminal --title='Robot Setup' -- bash -c '{cmd}'",
                shell=True
            )
            self.finished.emit(True)

        except Exception as e:
            self.error.emit(f"Setup error: {str(e)}")
            self.finished.emit(False)


class ExecuteCodeThread(WorkerThread):
    """Thread zum Bauen und Ausführen des generierten Codes"""

    def run(self):
        try:
            build_cmd = (
                f'cd {self.workspace_path} && '
                f'source /opt/ros/humble/setup.bash && '
                f'source install/setup.bash && '
                f'colcon build --packages-select ur10e_hl_interface && '
                f'echo && echo "=== Build complete ===" && echo && '
                f'read -p "Build erfolgreich? [j/n]: " response && '
                f'if [ "$response" = "j" ]; then '
                f'  echo && echo "=== Starting execution ===" && '
                f'  ros2 run ur10e_hl_interface clipfix_bewegung; '
                f'else '
                f'  echo "Execution aborted."; '
                f'fi; '
                f'echo && echo "Process finished. Press ENTER to close..."; read'
            )

            subprocess.Popen(
                f"gnome-terminal --tab --title='Build & Execute' -- bash -c '{build_cmd}'",
                shell=True
            )

            self.output.emit("Build and execution started in terminal.")
            self.finished.emit(True)

        except Exception as e:
            self.error.emit(f"Execution error: {str(e)}")
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
                self.output.emit(f"Using Claude: {self.model}")

            elif self.model.startswith("ollama:"):
                # Ollama (Netzwerk-Server)
                actual_model = self.model.replace("ollama:", "")
                client = interactLLM(
                    api_key=None,
                    model=actual_model,
                    api_type="ollama",
                    base_url=self.ollama_base_url
                )
                self.output.emit(f"Using Llama (network): {actual_model}")

            else:
                # OpenAI Cloud
                client = interactLLM(
                    api_key=self.api_key,
                    model=self.model,
                    api_type="openai"
                )
                self.output.emit(f"Using OpenAI: {self.model}")

            # Generiere Antwort (blockierend, aber in separatem Thread)
            response = client.generate_response(
                prompt=self.full_prompt,
                max_tokens=10000,
                temperature=0.3  # Niedrig für deterministische Ausgaben
            )

            # Sende Antwort zurück zur GUI
            self.response_ready.emit(response, self.level)

        except Exception as e:
            self.error.emit(f"Error generating response: {str(e)}")


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
        self.current_prompt = ""
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
        """Erstellt den Hauptbereich mit Output und Info-Displays"""
        main_layout = QHBoxLayout()

        # Links: LLM-Output
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setPlaceholderText("LLM-generated Outputs")
        self.output_text.setStyleSheet("""
            QTextEdit {
                background-color: white;
                color: black;
                border-radius: 5px;
                font-family: Arial;
                font-size: 12px;
            }
        """)

        # Rechts: Info-Displays
        info_layout = QVBoxLayout()

        # Verfügbare Komponenten
        comp_group = self._create_group_box("Available Components")
        self.components_display = ComponentsDisplay()
        comp_group.layout().addWidget(self.components_display)

        # Schaltschrank-Status
        cabinet_group = self._create_group_box("Current Cabinet Configuration")
        self.cabinet_display = CabinetStateDisplay()
        cabinet_group.layout().addWidget(self.cabinet_display)

        info_layout.addWidget(comp_group, 1)
        info_layout.addWidget(cabinet_group, 1)

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
        """Erstellt den Eingabebereich"""
        self.input_text = QTextEdit()
        self.input_text.setPlaceholderText("Describe the desired robot task...")
        self.input_text.setFixedHeight(100)
        self.input_text.setStyleSheet("""
            QTextEdit {
                background-color: white;
                color: black;
                border-radius: 5px;
                font-family: Arial;
                font-size: 12px;
            }
        """)
        layout.addWidget(self.input_text)

    def _create_options(self, layout: QVBoxLayout):
        """Erstellt die Level-1-Optionen"""
        self.options_group = QGroupBox("Options After Level 1")
        self.options_group.setStyleSheet("""
            QGroupBox {
                color: white;
                background-color: rgba(255, 255, 255, 0.1);
                border-radius: 5px;
                padding: 8px;
            }
        """)

        options_layout = QHBoxLayout()
        self.radio_proceed = QRadioButton("Proceed to Code Generation")
        self.radio_proceed.setChecked(True)
        self.radio_regenerate = QRadioButton("Regenerate Analysis")
        self.radio_edit = QRadioButton("Edit Prompt")

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
        self.btn_proceed = HoverButton("Proceed", "#28a745", "#218838")
        self.btn_setup = HoverButton("Setup Robot", "#007bff", "#0056b3")
        self.btn_execute = HoverButton("Execute Code", "#17a2b8", "#138496")
        self.btn_reset = HoverButton("Reset State", "#ffc107", "#e0a800")
        self.btn_close = HoverButton("Close", "#DC3545", "#a71d2a")

        # Model-Auswahl
        self.model_select = QComboBox()
        self.model_select.addItems([
            # Cloud-Modelle
            "gpt-4o",
            "gpt-4",
            "claude-opus-4-0",
            "claude-sonnet-4-0",
            "claude-3-7-sonnet-latest",
            # Lokale Modelle (Netzwerk-Server 134.147.216.152)
            "ollama:llama3.1:70b",  # Allzweck, beste Qualität
            "ollama:deepseek-r1:32b",  # Reasoning, komplex denken
            "ollama:qwen3-coder:30b",  # Code-Generierung
            "ollama:gemma3:27b"  # Google's Modell
        ])
        self.model_select.setCurrentIndex(5)  # Llama as default
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
                       self.btn_reset, self.model_select, self.btn_close]:
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
        self.timer.start(2000)  # Alle 2 Sekunden

        # Initiale Aktualisierung
        self._refresh_displays()

    def _refresh_displays(self):
        """Aktualisiert die Anzeigen"""
        try:
            # Parser-State vor dem Update neu laden
            from aml_prompt_parser import get_parser
            parser = get_parser()
            parser.parse_state()  # Nur State neu laden, nicht die ganze Config

            self.components_display.setComponents(get_available_quantities())
            self.cabinet_display.updateState()
        except Exception as e:
            print(f"Error updating displays: {e}")

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
            self.status_label.setText(f"{self._get_status_text()} - Computing...")
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
                self.input_text.setPlainText(self.current_prompt)
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
            self.btn_setup.setText("Setup Robot")
            self.btn_setup.setColor("#007bff")
            return

        # Thread als Instanzvariable speichern, damit er nicht vorzeitig zerstört wird
        self.setup_thread = SetupRobotThread(self.config.WORKSPACE_PATH)
        self.setup_thread.finished.connect(lambda success: self._on_setup_finished(success))
        self.setup_thread.start()

    def handle_execute(self):
        """Führt den generierten Code aus"""
        # Thread als Instanzvariable speichern
        self.execute_thread = ExecuteCodeThread(self.config.WORKSPACE_PATH)
        self.execute_thread.output.connect(lambda msg: self.show_output(msg, "system"))
        self.execute_thread.error.connect(lambda msg: self.show_output(msg, "error"))
        self.execute_thread.finished.connect(lambda success: QTimer.singleShot(3000, self._refresh_displays))
        self.execute_thread.start()

    def handle_reset(self):
        """Setzt den Schaltschrank-Zustand zurück"""
        reply = QMessageBox.question(
            self, 'Reset State',
            'Are you sure you want to reset the cabinet state?\n\n'
            'All rails and components will be removed from the cabinet.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            state_file = os.path.join(
                self.config.WORKSPACE_PATH,
                "src/ur10e_hl_interface/config/SchaltschrankZustand.aml"
            )
            try:
                if os.path.exists(state_file):
                    os.remove(state_file)
                    self.show_output("Cabinet state reset successfully!", "system")
                else:
                    self.show_output("No state file found to reset.", "system")
                self._refresh_displays()
            except Exception as e:
                self.show_output(f"Error resetting state: {str(e)}", "error")

    def generate_response(self, level: int):
        """Generiert eine LLM-Antwort für das angegebene Level (non-blocking)"""
        # Validierung für Level 1
        if level == 1:
            user_text = self.input_text.toPlainText().strip()
            if not user_text:
                self.show_output("Please enter a task description.", "error")
                return

            self.current_prompt = user_text

        self.show_output(f"Generating Level {level} response...", "user")
        self._show_computing(True)

        # Hole aktuelle System-Prompts
        system_prompts = {
            1: generate_level1_prompt(),
            2: generate_level2_prompt()
        }

        # Erstelle vollständigen Prompt
        full_prompt = f"{system_prompts[level]}\n\nUser Request: {self.current_prompt}"

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
            self.btn_proceed.setText("Start New")

        self._show_computing(False)

    def _handle_llm_error(self, error_msg: str):
        """Callback wenn LLM-Fehler auftritt"""
        self._show_computing(False)
        self.show_output(error_msg, "error")

    def reset_to_level1(self):
        """Setzt die Anwendung auf Level 1 zurück"""
        self.current_level = 1
        self.responses = {1: "", 2: ""}
        self.generated_code = ""
        self.output_text.clear()
        self.status_label.setText(self._get_status_text())
        self._update_status_style()
        self.options_group.setVisible(False)
        self.btn_execute.setEnabled(False)
        self.btn_proceed.setText("Proceed")
        self._refresh_displays()

    def _save_generated_code(self):
        """Speichert den generierten Code"""
        try:
            # Entferne Markdown-Formatierung falls vorhanden
            code = self.generated_code.strip()
            if code.startswith("```"):
                lines = code.split('\n')
                if len(lines) > 2:
                    code = '\n'.join(lines[1:-1])

            file_path = os.path.join(
                self.config.WORKSPACE_PATH,
                "src/ur10e_hl_interface/src/clipfix_bewegung.cpp"
            )

            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'w') as f:
                f.write(code)

            self.show_output(f"Code saved to: {file_path}", "system")

        except Exception as e:
            self.show_output(f"Error saving file: {str(e)}", "error")

    def _on_setup_finished(self, success: bool):
        """Callback nach Robot-Setup"""
        if success:
            self.robot_ready = True
            self.btn_setup.setText("Robot Ready ✓")
            self.btn_setup.setColor("#28a745")
            self.show_output("Robot setup complete! Ready to execute code.", "system")
        else:
            self.btn_setup.setText("Setup Robot")
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