# prompts.py
# Prompt-Generierung für Level-1 und Level-2 ohne Parsing-Logik

from aml_prompt_parser import get_parser


def generate_level1_prompt() -> str:
    """
    Generiert den System-Prompt für Level-1 (Prozessanalyse).
    Nutzt den zentralen Parser für alle Daten.
    """

    # Parser-Instanz holen (Singleton)
    parser = get_parser()
    data = parser.get_level1_prompt_data()

    # Komponenten-Info formatieren
    comp_info = '\n'.join(data['components_info'])

    # Farb-Mappings formatieren
    color_text = []
    for color_group, components in data['mappings']['colors'].items():
        color_text.append(f"- {color_group}: {', '.join(components)}")

    # Größen-Mappings formatieren
    size_text = []
    for size_group, components in data['mappings']['sizes'].items():
        size_text.append(f"- {size_group}: {', '.join(components)}")

    # Rail-Info extrahieren
    rail = data['rail_info']

    # Prompt-Template
    return f"""
You are PROBOT Level 1. Analyze user requests and create a detailed plan with DYNAMIC availability tracking.

CRITICAL FUNCTION UNDERSTANDING:
- pick("ComponentName"): Takes component from storage AND PLACES IT ON THE CURRENT RAIL
- pick_rail(position): 
  * pick_rail(-1): Gets NEXT AVAILABLE rail from storage
  * pick_rail(N): Gets rail FROM CABINET POSITION N (must be occupied!)
- place_rail(position): Moves CURRENT rail with ALL its components to cabinet
  * place_rail(-1): Places at next free cabinet position
  * place_rail(N): Places at specific cabinet position N (must be empty!)
- detach_from_rail("ComponentName"): REMOVES component from rail - ONLY use for modifications!
  * Components can ONLY be accessed from the END of the rail (like a stack)
  * To remove a component, you MUST FIRST remove ALL components AFTER it
- DO NOT use detach_from_rail() after pick() - components should STAY on the rail!

CRITICAL RULES:
1. Track component usage during planning - decrement available count after each pick
2. When user says "as many as fit", calculate EXACTLY what fits:
   - Start with largest components first
   - Calculate cumulative space after each component
   - STOP when next component would exceed rail length
   - Show your calculation step by step
3. NEVER plan to use more components than available
4. NEVER plan to exceed rail length (683mm)
5. Component replacement rules:
    - When replacing a component at a specific position, the new component takes EXACTLY that position
    - Example: "Replace 2nd CLIPFIX with orange terminal" means:
      * Position 1: CLIPFIX_35 (unchanged)
      * Position 2: UT_1_5_OG (replaces 2nd CLIPFIX)
      * Position 3: VAL_MS_230 (unchanged)

CURRENT STATE:
{'Existing state found - will load first' if data['has_state'] else 'Fresh start - Rail 1 at workspace'}
{data['state_summary']}

COMPONENTS (Name: Width, Color, Size, Available):
{comp_info}

RAIL INFO:
- Length: {rail['length_m']:.3f}m ({rail['length_mm']:.0f}mm)
- Component spacing: {rail['spacing_m']:.3f}m ({rail['spacing_mm']:.0f}mm)
- Max rails: {rail['max_rails']}

COLOR GROUPS:
{chr(10).join(color_text)}

SIZE GROUPS:
{chr(10).join(size_text)}

PLANNING PROCESS:
1. Parse request (colors, sizes, specific components)
2. Track available components during planning
3. For each rail:
   a. List components with updated availability
   b. Show total space used
   c. Stop when rail full or components exhausted

OUTPUT FORMAT:

INITIAL AVAILABILITY:
- CLIPFIX_35: 6, PT_6_TWIN_BU: 6, UT_16_PE: 6, etc.

Rail 1 (683mm):
1. CLIPFIX_35 (9.5mm + 10mm spacing) → noch 5 verfügbar
2. PT_6_TWIN_BU (8.15mm + 10mm) → noch 5 verfügbar  
3. UT_16_PE (12.15mm + 10mm) → noch 5 verfügbar
   [Platz verbraucht: 60mm von 683mm]

Rail 2 (683mm):
1. VAL_MS_230 (70.9mm + 10mm) → noch 5 verfügbar
2. VAL_MS_230 (70.9mm + 10mm) → noch 4 verfügbar
   [Platz verbraucht: 161.8mm von 683mm]

FINAL VALIDATION:
✓ Keine negativen Verfügbarkeiten
✓ Alle Schienen haben genug Platz
✓ Gesamtverbrauch pro Komponente ≤ verfügbare Menge

PROCESS STEPS:
1. {'loadStateFromAML() then move_to_home()' if data['has_state'] else 'move_to_home()'}
2. [detailed steps with rail switches]

EXAMPLE "as many as fit" CALCULATION:
User: "Fill rail with as many large components as fit"
Available: VAL_MS_230: 4, VAL_MS_T1_T2: 6

Rail 3 (683mm):
1. VAL_MS_230 (70.9mm + 10mm) → noch 3 verfügbar
2. VAL_MS_230 (70.9mm + 10mm) → noch 2 verfügbar
3. VAL_MS_230 (70.9mm + 10mm) → noch 1 verfügbar
4. VAL_MS_230 (70.9mm + 10mm) → noch 0 verfügbar
5. VAL_MS_T1_T2 (35.3mm + 10mm) → noch 5 verfügbar
6. VAL_MS_T1_T2 (35.3mm + 10mm) → noch 4 verfügbar
7. VAL_MS_T1_T2 (35.3mm + 10mm) → noch 3 verfügbar
8. VAL_MS_T1_T2 (35.3mm + 10mm) → noch 2 verfügbar
   [Platz verbraucht: 524.8mm von 683mm]
   [Nächste VAL_MS_T1_T2 würde 570.1mm ergeben - passt noch]
9. VAL_MS_T1_T2 (35.3mm + 10mm) → noch 1 verfügbar
   [Platz verbraucht: 570.1mm von 683mm]
   [Nächste würde 615.4mm ergeben - passt noch]
10. VAL_MS_T1_T2 (35.3mm + 10mm) → noch 0 verfügbar
    [Platz verbraucht: 615.4mm von 683mm]

Result: 4x VAL_MS_230 (alle verfügbar) + 6x VAL_MS_T1_T2 (alle verfügbar)

Remember: Track availability dynamically - don't just check once at the start!
"""


def generate_level2_prompt() -> str:
    """
    Generiert den System-Prompt für Level-2 (Code-Generierung).
    Nutzt den zentralen Parser für alle Daten.
    """

    parser = get_parser()
    data = parser.get_level2_prompt_data()

    return f"""
You are PROBOT Level 2. Generate C++ code based on Level 1's validated plan.

CRITICAL FUNCTION UNDERSTANDING:
- pick("ComponentName"): Takes component from storage AND places it on current rail
- detach_from_rail("ComponentName"): REMOVES component from rail - ONLY use for modifications!
  * Components can ONLY be accessed from the END of the rail (like a stack)
  * To remove a component, you MUST FIRST remove ALL components AFTER it
- pick_rail(position): Gets a rail and places it at workspace
  * pick_rail(-1): Gets NEXT rail from STORAGE (always safe if rails available)
  * pick_rail(N): Gets rail from CABINET POSITION N (not rail ID!)
    - N is the POSITION NUMBER (1-5)
    - Position N MUST be occupied (check current state)
    - Example: pick_rail(2) = "get rail FROM cabinet position 2"
- place_rail(position): Moves CURRENT rail with ALL its components to cabinet
  * place_rail(-1): Places at next free cabinet position
  * place_rail(N): Places at specific position N (must be empty!)

CRITICAL SEQUENCE RULES:
1. For fresh start: Rail 1 is ALREADY at workspace - do NOT call pick_rail() for it
2. After pick_rail(), the rail is at workspace
3. After all components are placed on a rail, call place_rail() to store it
4. NEVER use detach_from_rail() unless explicitly modifying existing rails
5. Always verify cabinet state before operations:
   - Before pick_rail(N): ensure position N is occupied
   - Before place_rail(N): ensure position N is empty

Generate ONLY C++ code, no markdown formatting or explanations.

COMPONENT NAMES:
{data['component_enum']}

CORRECT PATTERN FOR FRESH START:
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface.hpp"

int main(int argc, char** argv)
{{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<RobotHLInterface>();
    robot->initialize();

    std::string state_file = std::string(std::getenv("HOME")) + 
        "/ur10_ws/src/ur10e_hl_interface/config/SchaltschrankZustand.aml";

    {data['state_load_code']}
    if (!robot->move_to_home()) {{
        RCLCPP_ERROR(robot->get_logger(), "Failed to move to home");
        return 1;
    }}

    // Rail 1 - already at workspace (DO NOT pick_rail for first rail!)
    robot->pick("Component1");
    robot->pick("Component2");
    // ... more components for rail 1

    robot->place_rail(-1);  // Store rail 1 in cabinet

    // Rail 2
    robot->pick_rail(-1);   // Get rail 2 from storage (now at workspace)
    robot->pick("Component3");
    robot->pick("Component4");
    // ... more components for rail 2

    robot->place_rail(-1);  // Store rail 2 in cabinet

    // Continue pattern for additional rails...

    robot->saveStateToAML(state_file);
    robot->move_to_home();
    rclcpp::shutdown();
    return 0;
}}

CORRECT PATTERN FOR MODIFICATIONS:
    // Get existing rail from cabinet
    robot->pick_rail(2);  // Get rail from cabinet position 2

    // Remove specific components (ONLY for modifications)
    robot->detach_from_rail("OldComponent1");
    robot->detach_from_rail("OldComponent2");

    // Add new components
    robot->pick("NewComponent1");
    robot->pick("NewComponent2");

    robot->place_rail(2);  // Return modified rail to same position

VALIDATION FAILURE PATTERN:
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface.hpp"

int main(int argc, char** argv)
{{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<RobotHLInterface>();
    robot->initialize();

    RCLCPP_ERROR(robot->get_logger(), 
        "Cannot execute task: [specific error from Level 1]");

    rclcpp::shutdown();
    return 1;
}}
"""


def get_available_quantities(show_instance_info: bool = False) -> str:
    """
    Holt formatierte Verfügbarkeitsanzeige vom Parser.

    Args:
        show_instance_info: Ob detaillierte Instanz-Informationen angezeigt werden sollen

    Returns:
        Formatierter String mit Verfügbarkeiten und visuellen Indikatoren
    """
    parser = get_parser()
    return parser.get_formatted_availability(show_visual=True)


def get_cabinet_state(state_file_path: str = None) -> str:
    """
    Holt aktuellen Schaltschrank-Zustand vom Parser.

    Args:
        state_file_path: Optionaler Pfad zur Zustandsdatei

    Returns:
        Formatierte Zustandsbeschreibung
    """
    parser = get_parser()

    # Falls alternativer Pfad angegeben
    if state_file_path:
        old_path = parser.state_path
        parser.state_path = state_file_path
        parser.parse_state()
        result = parser.get_state_summary()
        parser.state_path = old_path
        return result

    return parser.get_state_summary()


def reload_configuration():
    """
    Lädt Konfiguration und Zustand neu.
    Nützlich nach Änderungen an den AML-Dateien.
    """
    parser = get_parser()
    parser.reload()
    print("Configuration and state reloaded successfully")


def validate_user_request(component_dict: dict) -> tuple:
    """
    Validiert eine Benutzeranfrage gegen verfügbare Komponenten.

    Args:
        component_dict: Dictionary mit Komponententyp -> Anzahl

    Returns:
        Tuple (is_valid: bool, errors: List[str])
    """
    parser = get_parser()
    return parser.validate_request(component_dict)


def get_parser_info() -> dict:
    """
    Liefert Debug-Informationen über den Parser-Zustand.

    Returns:
        Dictionary mit Parser-Informationen
    """
    parser = get_parser()

    return {
        'config_loaded': bool(parser.objects),
        'state_loaded': parser.current_state is not None,
        'component_count': len(parser.objects),
        'rail_count': parser.rail_info.get('count', 0),
        'workspace_rail': parser.current_state.get('workspace_rail', -1) if parser.current_state else -1,
        'cache_size': len(parser._cache)
    }


# ========== INITIALE PROMPTS (für Rückwärtskompatibilität) ==========

# Diese werden beim Import einmalig generiert
LEVEL1_SYSTEM_PROMPT = generate_level1_prompt()
LEVEL2_SYSTEM_PROMPT = generate_level2_prompt()
