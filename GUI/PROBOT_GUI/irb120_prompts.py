# irb120_prompts.py
# Prompt generation for IRB 120 robot system
# Uses dynamic AML data for cube positions and colors

from irb120_aml_parser import get_irb120_parser


def generate_level1_prompt(cell_setup_text: str = "", robot_task_text: str = "") -> str:
    """
    Generate Level 1 system prompt for task analysis.
    Dynamically includes current cube positions from AML.

    Args:
        cell_setup_text: User description for cell configuration changes (optional)
        robot_task_text: User description for robot task
    """
    parser = get_irb120_parser()
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

**Example: Moving a traffic light from C3 to C4**
Stack at C3: cube_0 (green, level 0) -> cube_1 (yellow, level 1) -> cube_2 (red, level 2)

Step 1 - Disassemble (top to bottom):
1. PickAndPlace("cube_2", "D3", 0)  // Pick red (top), place at temp
2. PickAndPlace("cube_1", "D4", 0)  // Pick yellow (now top), place at temp
3. PickAndPlace("cube_0", "C4", 0)  // Pick green (now top), place at destination

Step 2 - Reassemble (bottom to top):
4. PickAndPlace("cube_1", "C4", 1)  // Stack yellow on green
5. PickAndPlace("cube_2", "C4", 2)  // Stack red on yellow

**RULE: Before picking a cube, check if it has cubes on top. If yes, move those first!**

=== CRITICAL: GRIPPER CLEARANCE CONSTRAINT ===

**The gripper is large and requires clearance space in front of each position!**

For any position XN (where N is the row number 2-5):
- Position X(N-1) MUST be FREE to access XN
- Example: To pick/place at A4 → A3 must be free
- Example: To pick/place at B2 → B1 must be free
- Example: To pick/place at C3 → C2 must be free

**Row 1 positions (A1, B1, C1, D1, E1) are ALWAYS accessible** - nothing is in front of them.

**PLANNING IMPLICATIONS:**
1. When moving cubes, consider the order to avoid blocking yourself
2. When building a "traffic light" (Ampel), place from BACK to FRONT:
   - Place at X3 first, then X2, then X1 (if building toward front)
3. If position in front is occupied, move that cube first!

**Example - Building traffic light at C column:**
BAD ORDER (will fail):
1. Place red at C1 ✓
2. Place yellow at C2 ✗ (C1 has red, blocks gripper!)

GOOD ORDER:
1. Place green at C3 ✓ (C2 free)
2. Place yellow at C2 ✓ (C1 free)
3. Place red at C1 ✓ (always accessible)

**Example - Moving cube from A4 when A3 is occupied:**
1. First move the cube at A3 to a temp position
2. Then pick from A4
3. Optionally move A3's cube back if needed

**INITIAL CUBE PLACEMENT BEST PRACTICES (for AddCube):**

When using AddCube() to set up the scene, follow these rules to avoid blocking issues:
1. **Prefer row 1 positions** (A1, B1, C1, D1, E1) - always accessible
2. **If using other rows**, ensure cubes don't block each other:
   - BAD: Green at C4, Yellow at C5 (C4 blocks C5)
   - GOOD: Yellow at C4, Green at C5 (pick yellow first, then C4 is free for green)
3. **Plan the pick order** - place cubes so earlier picks free later ones
4. The system will auto-move blocking cubes, but proper planning is more efficient

=== GRID LAYOUT ===

{data['grid_text']}

=== CURRENT SCENE STATE ===

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

VALIDIERUNG:
[Prüfung ob Würfel existieren, Positionen gültig sind]

PLAN:
Phase 1 - Zellkonfiguration:
1. ClearAllCubes()  // IMMER zuerst aufrufen!
2. AddCube(...) für jeden gewünschten Würfel
...

Phase 2 - Roboter-Aufgabe:
1. [Schritt mit PickAndPlace falls nötig]
...

ZUSAMMENFASSUNG:
[Kurze Beschreibung was passieren wird]

=== BENUTZERANFRAGE ===

{user_request_section if user_request_section else "Keine spezifische Anfrage. Bitte warten Sie auf Benutzereingabe."}

Analysiere diese Anfrage und erstelle einen detaillierten Plan auf Deutsch.
"""


def generate_level2_prompt() -> str:
    """
    Generate Level 2 system prompt for C++ code generation.
    Includes current scene state for reference.
    """
    parser = get_irb120_parser()
    data = parser.get_prompt_data()

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

{data['objects_text']}

=== CODE TEMPLATE ===

Generate ONLY pure C++ code. NO markdown formatting, NO explanations.
Use this exact structure:

#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface_simple.hpp"

int main(int argc, char** argv)
{{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<RobotHLInterfaceSimple>();

    RCLCPP_INFO(robot->get_logger(), "Starting task execution...");

    // Initialize MoveIt and load AML configuration
    if (!robot->initialize()) {{
        RCLCPP_ERROR(robot->get_logger(), "Failed to initialize robot interface");
        rclcpp::shutdown();
        return 1;
    }}

    // === PHASE 1: CELL SETUP (instant operations, no robot movement) ===

    // ALWAYS clear all cubes first, then add the desired ones
    // robot->ClearAllCubes();  // Remove everything first!

    // Add new cubes (on table)
    // std::string new_cube = robot->AddCube("E1", "Blau");
    // RCLCPP_INFO(robot->get_logger(), "Created cube: %s", new_cube.c_str());

    // Add stacked cubes (instant, no robot movement) - e.g. traffic light
    // robot->AddCube("C3", "Gruen", 0); // Green on table (bottom)
    // robot->AddCube("C3", "Gelb", 1);  // Yellow on green (middle)
    // robot->AddCube("C3", "Rot", 2);   // Red on yellow (top)

    // === PHASE 2: ROBOT TASK (robot movement) ===

    // Robot picks and places cube on floor (level 0)
    // if (!robot->PickAndPlace("cube_0", "A3")) {{
    //     RCLCPP_ERROR(robot->get_logger(), "PickAndPlace failed!");
    //     return 1;
    // }}

    // Robot stacks cube on top of another cube (level 1)
    // if (!robot->PickAndPlace("cube_1", "A3", 1)) {{
    //     RCLCPP_ERROR(robot->get_logger(), "Stack at level 1 failed!");
    //     return 1;
    // }}

    // Robot ejects cube to ramp (zur Rampe/Rutsche)
    // if (!robot->PickAndPlace("cube_0", "X")) {{
    //     RCLCPP_ERROR(robot->get_logger(), "Eject to ramp failed!");
    //     return 1;
    // }}

    // Move to home when done
    robot->moveToHome();

    RCLCPP_INFO(robot->get_logger(), "Task completed successfully!");
    rclcpp::shutdown();
    return 0;
}}

=== IMPORTANT RULES ===

1. Generate ONLY the C++ code, no markdown code blocks
2. Check return values and log errors appropriately
3. Use the exact object names from the current scene
4. For new cubes, store the returned name if needed later
5. End with moveToHome() unless specified otherwise
6. If the plan is invalid, generate code that logs an error and returns 1
7. **GRIPPER CLEARANCE**: For positions X2-X5, position X(N-1) must be free!
   - A4 needs A3 free, B2 needs B1 free, etc.
   - X1 row positions are always accessible
   - The system auto-moves blocking cubes, but proper planning is more efficient
8. **CUBE PLACEMENT ORDER**: When adding cubes, prefer row 1 positions (A1-E1).
   If placing at higher rows, consider pick order to avoid mutual blocking.
   Example: If building Ampel from cubes at different positions:
   - Place cube that will be picked LAST at higher row (e.g., C5)
   - Place cube that will be picked FIRST at lower row (e.g., C4)

=== ERROR HANDLING PATTERN ===

For validation failures, generate:

#include <rclcpp/rclcpp.hpp>
#include "ur10e_hl_interface/robot_hl_interface_simple.hpp"

int main(int argc, char** argv)
{{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<RobotHLInterfaceSimple>();

    RCLCPP_ERROR(robot->get_logger(), "Cannot execute task: [specific error]");

    rclcpp::shutdown();
    return 1;
}}
"""


def get_scene_summary() -> str:
    """Get a brief summary of the current scene for display"""
    parser = get_irb120_parser()
    objects = parser.get_current_objects()

    if not objects:
        return "Scene: Empty (no cubes)"

    lines = [f"Scene: {len(objects)} cube(s)"]
    for obj in objects:
        lines.append(f"  {obj['name']}: {obj['location']} ({obj['color']})")

    return '\n'.join(lines)


def get_available_positions() -> str:
    """Get formatted list of available positions"""
    parser = get_irb120_parser()
    free = parser.get_free_positions()

    if not free:
        return "No free positions available"

    return f"Free positions: {', '.join(free)}"


def reload_scene():
    """Reload scene data from AML"""
    parser = get_irb120_parser()
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
