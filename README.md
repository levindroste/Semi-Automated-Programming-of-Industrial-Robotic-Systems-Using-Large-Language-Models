# Semi-Automated Programming of Industrial Robotic Systems Using Large Language Models

A framework for programming industrial robots (ABB IRB-120, UR10e) using natural language instructions and LLM-based code generation.

## Video Overview

A visual walkthrough of the framework, including setup, GUI, and task execution, is available here:
[Watch on YouTube](https://youtu.be/bkZYkbvItmU)

---

## Dual-Mode Operation

The system supports two distinct operation modes for different use cases:

### Würfel-Modus (Cube Mode)

Interactive cube manipulation on a 5×5 grid:

- **Grid System:** Coordinates A1-E5 (column A-E, row 1-5)
- **Stacking:** Level notation for stacked cubes (A1:1, A1:2, A1:3)
- **Colored Cubes:** red, green, blue, yellow, orange
- **Functions:**
  - `ClearAllCubes()` - Remove all cubes instantly
  - `AddCube(position, color, level)` - Spawn cube at grid position
  - `PickAndPlace(source, target)` - Robot moves cube between positions
  - `moveToHome()` - Return robot to home position

### Klemmen-Modus (Industrial Terminal Block Mode)

Industrial terminal block sorting for packaging automation:

- **5 Clamp Types:** Klemme1, Klemme2, Klemme3, Klemme4, Klemme5
- **5 Target Cartons:** Karton1, Karton2, Karton3, Karton4, Karton5
- **Predefined Packages:** PaketA, PaketB, etc. (preconfigured clamp combinations)
- **Real-time Tracking:** Availability tracking for each clamp type
- **Functions:**
  - `PickAndPlace(klemme_typ, karton_name)` - Pick clamp and place in carton
  - `moveToHome()` - Return robot to home position

---

## Installation (General Overview)

### 1. Install ROS2 Humble

Install **ROS2 Humble** on your system, including the standard desktop tools.
Make sure the ROS2 environment is properly sourced so that ROS tools and packages are available.

### 2. Install Gazebo, RViz & MoveIt2

Install the simulation and visualization stack:

- **Gazebo (Ignition)** for physics simulation
- **RViz** for visualization
- **MoveIt2** for motion planning

### 3. Install Python Dependencies

Install the necessary Python packages for running the project.
This includes standard packages (e.g., `requests`, `PyQt`, etc.) as well as any AI-related libraries.

### 4. Clone and Build the Project

Clone the repository and build using colcon:

```bash
git clone https://github.com/levindroste/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models.git
cd Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 5. Configure API Keys

Create a `.env` file in `GUI/PROBOT_GUI/` with your API keys:

```
OPENAI_API_KEY=your_key_here
ANTHROPIC_API_KEY=your_key_here
```

---

## Usage

### Start the Robot Simulation

```bash
# Würfel-Modus (default)
python3 launcher.py --cell-mode wuerfel

# Klemmen-Modus (industrial)
python3 launcher.py --cell-mode klemmen
```

### Start the GUI

```bash
python3 GUI/PROBOT_GUI/main.py
```

### Workflow

1. **Setup Robot Environment**
   - Run launcher.py to start Gazebo, RViz, and MoveIt2
   - Verify robot is upright and systems are initialized

2. **Enter Task Description**
   - Würfel-Mode: *"Place a red cube at A1 and a blue cube at B2"*
   - Klemmen-Mode: *"Pack 2x Klemme1 and 1x Klemme3 into Karton1"*

3. **Review Analysis**
   - System analyzes task and generates step-by-step plan
   - Options: Proceed / Regenerate / Edit

4. **Execute**
   - System generates and executes robot commands
   - Monitor execution in RViz

---

## Project Structure

```
Semi-Automated-Programming.../
├── src/
│   ├── ur10e_hl_interface/           # Main ROS2 package
│   │   ├── src/
│   │   │   ├── robot_hl_interface.cpp    # High-level robot interface
│   │   │   └── add_objects_node.cpp      # Scene object management
│   │   ├── include/
│   │   │   └── robot_hl_interface.hpp    # Interface header
│   │   └── config/
│   │       ├── wuerfel_config.aml        # Würfel-Modus configuration
│   │       └── klemmen_config.aml        # Klemmen-Modus configuration
│   │
│   ├── ur_with_gripper/              # Robot description (ABB IRB-120)
│   └── ur_with_gripper_moveit_config/# MoveIt2 configuration
│
├── GUI/PROBOT_GUI/                   # Python GUI application
│   ├── main.py                       # Main GUI (PyQt6)
│   ├── InterActLLM.py                # LLM interface (OpenAI/Claude/Ollama)
│   │
│   ├── wuerfel_prompts.py            # Würfel-Modus prompts
│   ├── wuerfel_aml_parser.py         # Würfel-Modus AML parser
│   │
│   ├── klemmen_prompts.py            # Klemmen-Modus prompts
│   ├── klemmen_aml_parser.py         # Klemmen-Modus AML parser
│   │
│   └── .env                          # API keys (not committed)
│
├── launcher.py                       # Automated ROS2 launcher
└── README.md                         # This file
```

---

## High-Level Interface API

### Würfel-Modus Functions

```python
# Scene setup (instant, no robot movement)
ClearAllCubes()                       # Remove all cubes
AddCube("A1", "red", level=0)         # Add cube at grid position

# Robot operations
PickAndPlace("A1", "B2")              # Move cube from A1 to B2
PickAndPlace("A1:2", "C3")            # Move stacked cube (level 2)
moveToHome()                          # Return to home position
```

### Klemmen-Modus Functions

```python
# Pick clamp and place in carton
PickAndPlace("Klemme1", "Karton1")    # Pick Klemme1, place in Karton1
PickAndPlace("Klemme3", "Karton2")    # Pick Klemme3, place in Karton2

# Multiple clamps of same type
PickAndPlace("Klemme1", "Karton1")    # First Klemme1
PickAndPlace("Klemme1", "Karton1")    # Second Klemme1 (auto-tracks availability)

moveToHome()                          # Return to home position
```

---

## Supported LLM Providers

The system supports multiple LLM backends via `InterActLLM.py`:

- **OpenAI:** GPT-4, GPT-4-turbo
- **Anthropic:** Claude 3 Opus, Sonnet, Haiku
- **Ollama:** Local models (Llama, Deepseek, Mistral, etc.)

Configure your preferred provider in the GUI or via environment variables.
