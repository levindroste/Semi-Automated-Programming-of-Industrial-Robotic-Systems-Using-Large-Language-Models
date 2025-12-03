# Claude Code Reference: Semi-Automated Industrial Robot Programming

## Project Overview

This project enables **semi-automated programming of industrial robotic systems using Large Language Models**. It combines LLMs with a UR10e collaborative robot to allow non-programmers to command complex robotic tasks through natural language descriptions.

### Core Concept
A **two-level LLM processing pipeline** transforms natural language task descriptions into executable C++ robot control code:
- **Level 1**: Task analysis and planning (understands user intent, validates requirements)
- **Level 2**: Code generation (converts validated plan to executable C++ code)

---

## ROS MCP Server Integration

**IMPORTANT**: This project has a ROS MCP server configured via [.mcp.json](.mcp.json) that provides direct ROS interaction capabilities.

### Before Making ROS-Related Changes

When Claude Code needs to work with ROS topics, services, or robot state, **you MUST first ensure rosbridge is running**:

1. **Check if rosbridge is running**:
   ```bash
   ps aux | grep rosbridge_websocket
   ```

2. **If not running, start rosbridge in the background**:
   ```bash
   source /opt/ros/humble/setup.bash && source ~/rosbridge_ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
   ```

   Or use the helper script:
   ```bash
   cd /home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models && ./start_rosbridge.sh &
   ```

3. **The MCP server will automatically connect** once rosbridge is running

### Available MCP Tools

Once rosbridge is running, you can use MCP tools to:
- Query ROS topics: "What topics are available?"
- Inspect robot state: "Show me the current robot joint positions"
- Monitor sensors: "What's the current camera feed?"
- Send commands: "Publish a velocity command"
- Call services: "Call the /get_planning_scene service"

### When to Start Rosbridge

Start rosbridge automatically (using Bash tool) when:
- User asks to interact with ROS topics or services
- You need to inspect the current robot state
- Making changes that require testing with live ROS nodes
- Debugging ROS communication issues

### Rosbridge Location
- **Built from source**: `~/rosbridge_ws`
- **MCP server location**: `~/ros-mcp-server`
- **Configuration**: See [MCP_SETUP.md](MCP_SETUP.md) for details

---

## Repository Structure

```
/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/
├── GUI/PROBOT_GUI/          # Python GUI application (PyQt6)
├── src/                     # ROS2 C++ packages for robot control
│   ├── ur10e_hl_interface/  # Core high-level robot interface
│   ├── ur_with_gripper/     # Robot URDF and launch files
│   └── [other packages]     # MoveIt config, control, gripper
├── launcher.py              # Automated ROS2 launcher
└── README.md                # Project documentation
```

---

## Key Technologies

### Robot Control Stack (C++)
- **ROS2 Humble**: Middleware and communication framework
- **MoveIt2**: Motion planning and collision avoidance
- **Gazebo Ignition**: Physics simulation
- **TinyXML2**: AML configuration parsing

### GUI and LLM Integration (Python)
- **PyQt6**: Desktop GUI framework
- **OpenAI SDK**: GPT-4 integration
- **Anthropic SDK**: Claude integration
- **Ollama API**: Local LLM support

### Configuration Format
- **AutomationML (AML)**: XML-based industrial automation standard for component definitions and state management

---

## Critical Files Reference

### GUI Application ([GUI/PROBOT_GUI/](GUI/PROBOT_GUI/))

| File | Purpose | Key Functions |
|------|---------|---------------|
| [main.py](GUI/PROBOT_GUI/main.py) | Main GUI with dual-level LLM processing | `process_level1()`, `process_level2()`, threading, state management |
| [InterActLLM.py](GUI/PROBOT_GUI/InterActLLM.py) | Unified LLM interface | Supports OpenAI, Claude, Ollama backends |
| [prompts.py](GUI/PROBOT_GUI/prompts.py) | System prompt generation | `generate_system_prompt_level1()`, `generate_system_prompt_level2()` |
| [aml_prompt_parser.py](GUI/PROBOT_GUI/aml_prompt_parser.py) | AML parser singleton | Component definitions, availability tracking |
| [aml_parser.py](GUI/PROBOT_GUI/aml_parser.py) | Legacy AML utilities | XML parsing helpers (currently open in IDE) |

### Robot Control ([src/ur10e_hl_interface/](src/ur10e_hl_interface/))

| File | Purpose | Key Components |
|------|---------|----------------|
| [src/robot_hl_interface.cpp](src/ur10e_hl_interface/src/robot_hl_interface.cpp) | Core robot control logic (2045 lines) | `pick()`, `place_rail()`, `pick_rail()`, `detach_from_rail()` |
| [include/robot_hl_interface.hpp](src/ur10e_hl_interface/include/robot_hl_interface.hpp) | Interface definitions | `RobotHLInterface` class, planning parameters |
| [src/add_objects_node.cpp](src/ur10e_hl_interface/src/add_objects_node.cpp) | Scene object spawning | Loads objects from AML into Gazebo |
| [src/clipfix_bewegung.cpp](src/ur10e_hl_interface/src/clipfix_bewegung.cpp) | **Generated code template** | LLM writes task code here |

### Configuration ([src/ur10e_hl_interface/config/](src/ur10e_hl_interface/config/))

| File | Purpose |
|------|---------|
| [AML-Datei-V04.aml](src/ur10e_hl_interface/config/AML-Datei-V04.aml) | Component definitions, geometric properties, robot parameters |
| [SchaltschrankZustand.aml](src/ur10e_hl_interface/config/SchaltschrankZustand.aml) | **Runtime state** (auto-generated, tracks component locations) |

---

## Core Robot Functions

The C++ interface provides these high-level operations:

```cpp
// Pick component and place on specified rail
bool pick(const std::string& component_name);

// Store rail in cabinet (position 1-5, or -1 for auto)
bool place_rail(int position = -1);

// Retrieve rail from cabinet
bool pick_rail(int position = -1);

// Remove component from rail and return to storage
bool detach_from_rail(const std::string& component_name);

// Return robot to home position
bool move_to_home();

// State persistence
bool loadStateFromAML(const std::string& state_file);
bool saveStateToAML(const std::string& state_file);
```

---

## System Architecture

### Level 1: Task Analysis Flow
```
User Input (natural language)
    ↓
LLM analyzes request
    ↓
- Parse component requirements
- Check availability via AML
- Calculate rail capacity (683mm max)
- Generate step-by-step plan
    ↓
Return validated plan to GUI
```

**Example Input**: "Place 3 CLIPFIX terminals and 2 red components on rail 1"

**Level 1 Output**: Structured plan with component IDs, placement order, validation checks

### Level 2: Code Generation Flow
```
Level 1 Plan (approved by user)
    ↓
LLM generates C++ code
    ↓
Code saved to clipfix_bewegung.cpp
    ↓
Built with colcon build
    ↓
Executed: ros2 run ur10e_hl_interface clipfix_bewegung
    ↓
Robot executes task
    ↓
State saved to SchaltschrankZustand.aml
```

---

## Component Management System

### Storage Architecture
- **Rails**: 683mm capacity, hold multiple components with 10mm spacing
- **Cabinet**: 5-position storage rack for completed rails
- **Component Types**: CLIPFIX terminals (various sizes/colors), mounting rails
- **State Tracking**: XML-based persistence across sessions

### Rail Capacity Calculation
The system automatically calculates if components fit on a rail:
```
Total Length = Σ(component_lengths) + (n-1) × 10mm spacing
Valid if: Total Length ≤ 683mm
```

---

## LLM Backend Configuration

The system supports three LLM backends via [InterActLLM.py](GUI/PROBOT_GUI/InterActLLM.py):

### 1. OpenAI API
- Models: GPT-4, GPT-4-turbo, etc.
- Configuration: API key required

### 2. Anthropic API
- Models: Claude 3.5 Sonnet, Claude 3 Opus, etc.
- Configuration: API key required

### 3. Ollama (Local)
- Models: Llama 3.1 70b, Deepseek, Mistral, etc.
- Configuration: Local server at http://localhost:11434
- No API key needed

---

## AutomationML (AML) Structure

AML files use XML to define industrial automation components:

### Component Definition Example
```xml
<InternalElement Name="CLIPFIX_terminal_3x_blue" ID="...">
    <Attribute Name="length">
        <Value>35.5</Value>
    </Attribute>
    <Attribute Name="approach_height">
        <Value>60.0</Value>
    </Attribute>
    <Attribute Name="color">
        <Value>blue</Value>
    </Attribute>
</InternalElement>
```

### State Tracking Example
```xml
<Attribute Name="location">
    <Value>rail_1_pos_2</Value>  <!-- Component currently on rail 1, position 2 -->
</Attribute>
<Attribute Name="is_available">
    <Value>false</Value>  <!-- Not available for picking -->
</Attribute>
```

---

## Development Workflow

### Making Changes to Robot Control
1. Edit C++ files in [src/ur10e_hl_interface/src/](src/ur10e_hl_interface/src/)
2. Build: `colcon build --packages-select ur10e_hl_interface`
3. Source: `source install/setup.bash`
4. Launch simulation or real robot

### Making Changes to GUI
1. Edit Python files in [GUI/PROBOT_GUI/](GUI/PROBOT_GUI/)
2. Activate venv: `source GUI/PROBOT_GUI/.venv/bin/activate`
3. Run: `python GUI/PROBOT_GUI/main.py`

### Modifying LLM Prompts
- Level 1 prompts: [prompts.py](GUI/PROBOT_GUI/prompts.py) → `generate_system_prompt_level1()`
- Level 2 prompts: [prompts.py](GUI/PROBOT_GUI/prompts.py) → `generate_system_prompt_level2()`
- Component info injection: [aml_prompt_parser.py](GUI/PROBOT_GUI/aml_prompt_parser.py)

### Adding New Components
1. Add component definition to [AML-Datei-V04.aml](src/ur10e_hl_interface/config/AML-Datei-V04.aml)
2. Include: name, length, approach_height, color, 3D model reference
3. System automatically detects new components

---

## Common Tasks

### Launch Robot Simulation
```bash
python launcher.py  # Automated launcher
# OR manually:
ros2 launch ur_with_gripper ur_sim_control.launch.py
```

### Run GUI Application
```bash
cd GUI/PROBOT_GUI
source .venv/bin/activate
python main.py
```

### Build Specific Package
```bash
colcon build --packages-select ur10e_hl_interface
source install/setup.bash
```

### Execute Generated Task Code
```bash
ros2 run ur10e_hl_interface clipfix_bewegung
```

### View Current Robot State
Check [SchaltschrankZustand.aml](src/ur10e_hl_interface/config/SchaltschrankZustand.aml) for current component locations and availability.

---

## Git Workflow

### Branches
- **main**: Stable production code
- **IRB-120**: Current development branch (experimental features)

### Recent Changes
- GUI refinements and improved user experience
- Updated AML configuration parsing
- Enhanced state management

---

## Important Constraints

### Rail System
- Maximum rail length: 683mm
- Minimum component spacing: 10mm
- Must validate total length before placement

### Cabinet Storage
- Exactly 5 positions available
- Must track which positions are occupied
- State persisted across sessions

### Component Availability
- Each component can only be in one location
- Availability tracked in real-time via AML
- Must update state after every operation

### Safety and Validation
- All movements validated by MoveIt2
- Collision checking enabled
- Home position recovery available

---

## Debugging Tips

### GUI Issues
- Check console output for LLM API errors
- Verify API keys in environment
- Test Ollama server: `curl http://localhost:11434/api/tags`

### Robot Control Issues
- Check ROS2 topics: `ros2 topic list`
- Monitor MoveIt planning: Launch RViz2
- Verify AML state file integrity: Check XML syntax

### LLM Output Issues
- Review prompts in [prompts.py](GUI/PROBOT_GUI/prompts.py)
- Check component availability in AML
- Validate Level 1 output before Level 2

---

## Key Design Principles

1. **Semi-Automated**: Human-in-the-loop validation between Level 1 and Level 2
2. **State Persistence**: All robot operations update AML state automatically
3. **Multi-Backend**: Flexible LLM backend selection (cloud or local)
4. **Component-Centric**: Everything defined and tracked in AutomationML
5. **ROS2 Native**: Full integration with standard ROS2 ecosystem

---

## External Resources

- **ROS2 Documentation**: https://docs.ros.org/en/humble/
- **MoveIt2 Documentation**: https://moveit.picknik.ai/humble/
- **AutomationML Standard**: https://www.automationml.org/
- **UR10e Robot Manual**: Universal Robots documentation

---

## Current File in IDE

**[aml_parser.py](GUI/PROBOT_GUI/aml_parser.py)** (351 lines)
- Legacy AML parsing utilities
- Functions for extracting component data from XML
- Being superseded by [aml_prompt_parser.py](GUI/PROBOT_GUI/aml_prompt_parser.py) singleton pattern
