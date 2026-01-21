# wuerfel_prompts.py
# Prompt generation for IRB 120 robot system - Würfel-Modus (Cube Mode)
# Uses dynamic AML data for cube positions and colors

from wuerfel_aml_parser import get_wuerfel_parser


def generate_level1_prompt(cell_setup_text: str = "", robot_task_text: str = "") -> str:
    """
    Generate Level 1 system prompt for task analysis.
    Dynamically includes current cube positions from AML.

    Args:
        cell_setup_text: User description for cell configuration changes (optional)
        robot_task_text: User description for robot task
    """
    parser = get_wuerfel_parser()
    data = parser.get_prompt_data()

    # Determine which modes are active
    cell_setup_mode = bool(cell_setup_text and cell_setup_text.strip())
    robot_task_mode = bool(robot_task_text and robot_task_text.strip())

    # Build the user request section based on what was provided
    user_request_section = ""
    if cell_setup_mode:
        user_request_section += f"""
=== PHASE 1: CELL SETUP REQUEST ===
{cell_setup_text}

"""
    if robot_task_mode:
        user_request_section += f"""=== PHASE 2: ROBOT TASK REQUEST ===
{robot_task_text}
"""

    # Build available functions section based on active modes
    available_functions = ""

    if cell_setup_mode:
        available_functions += """
**For CELL SETUP (instant operations):**

1. ClearAllCubes()
   - Removes ALL cubes from the scene INSTANTLY (no robot movement)
   - Safe to call even if there are no cubes
   - **ALWAYS call this FIRST before adding cubes!**

2. AddCube(grid_position, color, level=0)
   - Spawns a new cube at the specified grid position (instant, no robot)
   - Returns the name of the created cube (e.g., "cube_0", "cube_1")
   - grid_position: Where to spawn (e.g., "E1", "C3")
   - color: German color name (Schwarz, Rot, Gruen, Gelb, Blau, Grau, Orange)
   - level: Stack level (default 0 = table, 1 = on first cube, 2 = on second, etc.)
   - **DIRECT STACKING**: Create stacked cubes instantly without robot:
     AddCube("C3", "Gruen", 0)  # Green on table (bottom)
     AddCube("C3", "Gelb", 1)   # Yellow on green (middle)
     AddCube("C3", "Rot", 2)    # Red on yellow (top)

**CELL SETUP WORKFLOW:**
1. ClearAllCubes()           # First: Remove everything
2. AddCube(...) for each cube # Then: Add only the desired cubes

"""

    if robot_task_mode:
        available_functions += """**For ROBOT TASK (physical movement):**

3. PickAndPlace(object_name, goal_position, level=0)
   - Robot PHYSICALLY picks up a cube and moves it to a new position
   - object_name: The name of the cube (e.g., "wuerfel_rot", "cube_0")
   - goal_position: Target grid position (e.g., "A3", "B4") or "X" for eject
   - level: Stack level (default 0 = floor)
     * level=0: Place on floor/table
     * level=1: Stack on top of first cube (+3cm height)
     * level=2: Stack on top of second cube (+6cm height)
   - **STACKING**: To stack cubes, place first cube at level=0, then next at level=1, etc.
     Example: PickAndPlace("cube_0", "A1", 0) then PickAndPlace("cube_1", "A1", 1)
   - **EJECT TO RAMP "X"**: Robot picks cube and ejects it (zur Rampe/Rutsche)
     Use when user says: "zur Rampe bringen", "zur Rutsche", "auswerfen", "eject"

4. moveToHome()
   - Moves robot to home position

"""

    # Build restriction notice if only one mode is active
    restriction_notice = ""
    if robot_task_mode and not cell_setup_mode:
        restriction_notice = """
=== WICHTIGE EINSCHRÄNKUNG ===
**NUR ROBOTER-AUFGABEN ERLAUBT!**
Der Benutzer hat NUR "Roboter-Aufgaben beschreiben" verwendet.
- ❌ ClearAllCubes() ist NICHT erlaubt
- ❌ AddCube() ist NICHT erlaubt
- ✅ PickAndPlace() ist erlaubt
- ✅ moveToHome() ist erlaubt

Wenn der Benutzer Würfel hinzufügen oder entfernen möchte, muss er
"Zelle verändern" verwenden. Erkläre dies dem Benutzer höflich.

"""
    elif cell_setup_mode and not robot_task_mode:
        restriction_notice = """
=== WICHTIGE EINSCHRÄNKUNG ===
**NUR ZELLKONFIGURATION ERLAUBT!**
Der Benutzer hat NUR "Zelle verändern" verwendet.
- ✅ ClearAllCubes() ist erlaubt
- ✅ AddCube() ist erlaubt
- ❌ PickAndPlace() ist NICHT erlaubt (keine Roboterbewegung)
- ❌ moveToHome() ist NICHT erlaubt

Wenn der Benutzer den Roboter bewegen möchte, muss er
"Roboter-Aufgaben beschreiben" verwenden. Erkläre dies dem Benutzer höflich.

"""

    # Warning about scene state when cell setup is active
    current_state_warning = ""
    if cell_setup_mode:
        current_state_warning = """
⚠️ **ACHTUNG: Dies ist der AUSGANGSZUSTAND vor Phase 1!**
Nach ClearAllCubes() + AddCube() existieren NUR die neuen Würfel.
Die unten gelisteten Würfel werden in Phase 1 GELÖSCHT und durch neue ersetzt.
"""

    return f"""You are PROBOT Level 1 for the ABB IRB 120 robot system.
Your task is to analyze user requests and create a detailed step-by-step plan.

**IMPORTANT: Always respond in German (Deutsch). All output text must be in German.**
{restriction_notice}
=== ROBOT SYSTEM OVERVIEW ===

The IRB 120 robot operates on a 5x5 grid of positions where cubes can be placed.
The system has two types of operations:

**CELL SETUP (instant, no robot movement):**
- The user describes the COMPLETE desired cell state
- ALWAYS call ClearAllCubes() FIRST to remove everything
- THEN call AddCube() for each cube the user wants
- **IMPORTANT**: Cell setup = complete replacement using ClearAllCubes() + AddCube()

**ROBOT TASK (physical robot movement):**
- Pick up cubes and place them at different positions
- Eject cubes to the ramp

=== AVAILABLE FUNCTIONS ===
{available_functions}
=== CRITICAL: STACK PHYSICS ===

**You can ONLY pick the TOP cube of a stack!**

When moving stacked cubes:
1. You must pick cubes from TOP to BOTTOM (reverse order)
2. Use temporary positions for intermediate storage
3. Then rebuild the stack at the destination from BOTTOM to TOP

**Example: Moving a stack from x.n to x.n+1**
Stack at x.n: cube_0 (bottom) -> cube_1 (middle) -> cube_2 (top)

Step 1 - Disassemble (top to bottom):
1. PickAndPlace("cube_2", "x+1.n", 0)    // top → temp
2. PickAndPlace("cube_1", "x+1.n+1", 0)  // middle → temp
3. PickAndPlace("cube_0", "x.n+1", 0)    // bottom → Ziel

Step 2 - Reassemble (bottom to top):
4. PickAndPlace("cube_1", "x.n+1", 1)    // stack level 1
5. PickAndPlace("cube_2", "x.n+1", 2)    // stack level 2

**RULE: Before picking a cube, check if it has cubes on top. If yes, move those first!**

=== BUCHSTABEN UND FORMEN ===

**Buchstaben/Formen werden FLACH auf dem Grid gelegt - NICHT gestapelt!**
Jeder Würfel = eine Grid-Position (level=0)

**Buchstabe "L" (4 Würfel):**
Startpunkt x.n (obere linke Ecke)
- Stamm vertikal: x.n, x.n+1, x.n+2
- Fuß horizontal: x+1.n+2

**Pfeil (IMMER 6 Würfel: 4 Schaft + 2 Flügel):**

Benutzer gibt Start (x.n) und Ende an.

**Horizontaler Pfeil von x.n nach x+3.n:**
- Schaft: x.n → x+1.n → x+2.n → x+3.n (4 Positionen)
- Flügel: x+2.n-1, x+2.n+1 (über/unter dem vorletzten Würfel)
- Spitze zeigt zu x+3.n (Endpunkt)

**Vertikaler Pfeil von x.n nach x.n+3:**
- Schaft: x.n → x.n+1 → x.n+2 → x.n+3
- Flügel: x-1.n+2, x+1.n+2

⚠️ **VOR JEDEM PickAndPlace PRÜFEN:**
1. Ist Zielposition FREI? (Nicht in "Occupied positions" UND nicht durch AddCube belegt)
2. Nach Phase 1 sind ALLE AddCube-Positionen BELEGT!

=== GRID LAYOUT ===

{data['grid_text']}

=== CURRENT SCENE STATE ===
{current_state_warning}
{data['objects_text']}

Occupied positions: {', '.join(data['occupied']) if data['occupied'] else 'None'}
Free positions: {', '.join(data['free_positions'][:15])}{'...' if len(data['free_positions']) > 15 else ''}

=== VALID COLORS ===

{', '.join(data['colors'])}
Note: Use German color names in code (Rot, Gruen, Gelb, etc.)

=== ANALYSIS GUIDELINES ===

1. Identify what the user wants to accomplish
2. Check if referenced cubes exist in the current scene
3. Verify target positions are valid and free

4. **Separate the two phases clearly:**

   **PHASE 1 - CELL SETUP (if requested):**
   - User describes the COMPLETE desired cell state (not incremental changes!)
   - ALWAYS call ClearAllCubes() FIRST (even if scene is empty)
   - THEN use AddCube() to spawn the described cubes
   - These operations are instant, no robot movement
   - Example: "Eine Ampel auf C3" → ClearAllCubes(), then AddCube for red/yellow/green on C3

   **PHASE 2 - ROBOT TASK (if requested):**
   - User describes what the robot should DO physically
   - Use PickAndPlace() for moving cubes
   - Use PickAndPlace(cube, "X") to eject to ramp

5. Create a clear step-by-step plan with phases clearly separated

6. **Check stack order before picking (CRITICAL!):**
   - Identify which cube is on TOP of each stack
   - You can ONLY pick the topmost cube
   - Plan temporary storage for intermediate cubes
   - Choose free positions near the work area for temp storage
   - When moving a stack: disassemble top-to-bottom, reassemble bottom-to-top

=== AUSGABEFORMAT ===

AUFGABENVERSTÄNDNIS:
[Was der Benutzer erreichen möchte]

PLAN:
Phase 1 - Zellkonfiguration:
1. ClearAllCubes()
2. AddCube("Position", "Farbe", level) für jeden Würfel
...

WÜRFEL NACH PHASE 1:
[Liste ALLE Würfel die nach Phase 1 existieren - NUR diese sind für Phase 2 verfügbar!]
- cube_0: Position (Farbe)
- cube_1: Position (Farbe)
...

Phase 2 - Roboter-Aufgabe:
[Verwende NUR die Würfel aus "WÜRFEL NACH PHASE 1"!]
1. PickAndPlace("cube_X", "Ziel", level)
...

ZUSAMMENFASSUNG:
[Kurze Beschreibung]

=== BENUTZERANFRAGE ===

{user_request_section if user_request_section else "Keine spezifische Anfrage. Bitte warten Sie auf Benutzereingabe."}

Analysiere diese Anfrage und erstelle einen detaillierten Plan auf Deutsch.
"""


def generate_level2_prompt(cell_setup_mode: bool = False) -> str:
    """
    Generate Level 2 system prompt for C++ code generation.

    Args:
        cell_setup_mode: If True, indicates that Level 1 used cell setup (ClearAllCubes + AddCube).
                         In this case, the AML scene state is irrelevant - use Level 1 cube list instead.
    """
    parser = get_wuerfel_parser()
    data = parser.get_prompt_data()

    # Scene state section depends on whether cell setup was used
    if cell_setup_mode:
        scene_state_section = """⚠️ **WICHTIG: Cell Setup wurde in Level 1 verwendet!**
Die AML-Daten sind IRRELEVANT - nach ClearAllCubes() existieren NUR die neuen AddCube-Würfel.

**Verwende die Würfel aus der Level 1 Analyse (Abschnitt "WÜRFEL NACH PHASE 1")!**
Diese Würfel wurden durch AddCube() erstellt und haben Namen wie cube_0, cube_1, etc."""
    else:
        scene_state_section = data['objects_text']

    return f"""You are PROBOT Level 2 for the ABB IRB 120 robot system.
Generate C++ code based on the Level 1 analysis plan.

**IMPORTANT: Use German for all RCLCPP_INFO/RCLCPP_ERROR log messages.**

=== AVAILABLE FUNCTIONS ===

The RobotHLInterfaceSimple class provides these methods:

**For CELL SETUP (instant operations):**

1. void ClearAllCubes()
   - Removes ALL cubes from scene INSTANTLY (no robot movement)
   - Safe to call even if there are no cubes
   - **ALWAYS call this FIRST before adding cubes!**

2. std::string AddCube(const std::string& grid_position, const std::string& color = "Schwarz", int level = 0)
   - Spawns new cube INSTANTLY (no robot movement)
   - Returns the name of created cube (e.g., "cube_0")
   - Valid colors: Schwarz, Rot, Gruen, Gelb, Blau, Grau, Orange
   - level: Stack level (0=table, 1=on first cube, 2=on second, etc.)

**For ROBOT TASK (physical movement):**

3. bool PickAndPlace(const std::string& object_name, const std::string& goal_name, int level = 0)
   - Robot PHYSICALLY picks up cube and places at goal position
   - Returns true on success, false on failure
   - goal_name: grid position (e.g., "A3") or "X" for eject to ramp
   - level: Stack level (default 0)
     * 0 = place on floor/table
     * 1 = stack on first cube (+3cm)
     * 2 = stack on second cube (+6cm)
   - Use "X" to eject cube to ramp (zur Rampe/Rutsche)

4. bool moveToHome()
   - Moves robot to home position

=== CURRENT SCENE STATE (for reference) ===

{scene_state_section}

=== CODE TEMPLATE ===

Generate ONLY pure C++ code. NO markdown formatting, NO explanations.

#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface_simple.hpp"

int main(int argc, char** argv)
{{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<RobotHLInterfaceSimple>();
    RCLCPP_INFO(robot->get_logger(), "Starte Aufgabenausführung...");

    if (!robot->initialize()) {{
        RCLCPP_ERROR(robot->get_logger(), "Initialisierung fehlgeschlagen");
        rclcpp::shutdown();
        return 1;
    }}

    // === PHASE 1: CELL SETUP (instant, no robot movement) ===
    // robot->ClearAllCubes();
    // std::string cube_0 = robot->AddCube("A1", "Gruen", 0);

    // === PHASE 2: ROBOT TASK (physical movement) ===
    // if (!robot->PickAndPlace("cube_0", "B1", 0)) {{ return 1; }}

    robot->moveToHome();
    RCLCPP_INFO(robot->get_logger(), "Aufgabe erfolgreich abgeschlossen!");
    rclcpp::shutdown();
    return 0;
}}

=== BUCHSTABEN/FORMEN FLACH LEGEN ===

**Buchstaben/Formen werden FLACH auf dem Grid gelegt - NICHT gestapelt!**
Alle Würfel auf level=0, unterschiedliche Grid-Positionen.

Beispiel "L" (4 Würfel): A1, A2, A3 (Stamm) + B1 (Fuß)
Pfeil (6 Würfel): 4 Schaft + 2 Flügel (siehe Level 1 Analyse)

=== REGELN ===

1. NUR C++ Code ausgeben - keine Markdown-Blöcke, keine Erklärungen
2. Exakte Würfelnamen aus Level 1 Analyse verwenden (cube_0, cube_1, etc.)
3. PickAndPlace Rückgabewerte prüfen, bei Fehler return 1
4. Am Ende moveToHome() aufrufen
5. **CHECKLISTE**: Ist Zielposition FREI? AddCube-Positionen sind sofort BELEGT!
"""


def get_scene_summary() -> str:
    """Get a brief summary of the current scene for display"""
    parser = get_wuerfel_parser()
    objects = parser.get_current_objects()

    if not objects:
        return "Scene: Empty (no cubes)"

    lines = [f"Scene: {len(objects)} cube(s)"]
    for obj in objects:
        lines.append(f"  {obj['name']}: {obj['location']} ({obj['color']})")

    return '\n'.join(lines)


def get_available_positions() -> str:
    """Get formatted list of available positions"""
    parser = get_wuerfel_parser()
    free = parser.get_free_positions()

    if not free:
        return "No free positions available"

    return f"Free positions: {', '.join(free)}"


def reload_scene():
    """Reload scene data from AML"""
    parser = get_wuerfel_parser()
    parser.reload()
    print("Scene data reloaded from AML")


# ========== TEST ==========

if __name__ == "__main__":
    print("=== Level 1 Prompt Preview ===")
    print(generate_level1_prompt()[:2000])
    print("\n...\n")

    print("=== Level 2 Prompt Preview ===")
    print(generate_level2_prompt()[:2000])
    print("\n...\n")

    print("=== Scene Summary ===")
    print(get_scene_summary())
