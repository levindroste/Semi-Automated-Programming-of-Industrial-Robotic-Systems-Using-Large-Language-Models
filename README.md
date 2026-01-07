## 🎬 Video Overview

A visual walkthrough of the framework, including setup, GUI, and task execution, is available here:  
[Watch on YouTube](https://youtu.be/bkZYkbvItmU)

---

## 🔧 Installation (General Overview)

This guide gives a high-level overview of what needs to be installed and configured, without prescribing exact commands. You can follow the official documentation of each component for the detailed installation steps.

---

### 1. Install ROS2 Humble

Install **ROS2 Humble** on your system, including the standard desktop tools.  
Make sure the ROS2 environment is properly sourced so that ROS tools and packages are available.

---

### 2. Install Gazebo, RViz & MoveIt2

Install the simulation and visualization stack:

- **Gazebo (Ignition)** for physics simulation  
- **RViz** for visualization  
- **MoveIt2** for motion planning  

Ensure that all required ROS2 plugins and common dependencies are available.

---

### 3. Install Python Dependencies

Install the necessary Python packages for running the project.  
This includes standard packages (e.g., `requests`, `PyQt`, etc.) as well as any AI-related libraries you plan to use.  
Use your system’s package manager and `pip` according to your environment.

---

### 4. Clone and Build the Project

Clone the repository into a folder of your choice.  
Initialize ROS2 dependency management (e.g., via `rosdep`) and build the project using your preferred ROS2 build system (typically `colcon`).  
After building, ensure that the generated setup scripts are sourced so the ROS nodes become available.

---

### 5. Configure API Keys

In the file **`main.py`**, insert your own API keys where required.  
This includes keys for language-model providers or any external services the project interacts with.

---

### 6. (Optional) Adjust ROS Network Settings

If you want to connect the system to an external robot or operate within a larger multi-machine ROS network, you can change the ROS communication parameters in **`main.py`**:

```python
# ========== CENTRAL ROS CONFIGURATION ==========
# Adjust these values if working in a shared or multi-robot ROS network
ROS_DOMAIN_ID = 42
ROS_LOCALHOST_ONLY = True

---

## 🚀 Usage

### Start the System

```bash
cd ~/ur10_ws
python3 gui/main.py
```

### Workflow

1. **Setup Robot Environment**
   - Click "Setup Robot" to launch Gazebo, RViz, and MoveIt2
   - Verify robot is upright and systems are initialized

2. **Enter Task Description**
   - Example: *"Place 3 CLIPFIX terminals and 2 red components on rail 1"*
   - Click "Proceed"

3. **Review Level 1 Analysis**
   - System analyzes task and breaks it into steps
   - Options: Proceed / Regenerate / Edit Prompt

4. **Level 2 Code Generation**
   - System generates executable C++ code
   - Code is automatically saved

5. **Execute**
   - Click "Execute Code" to build and run
   - Monitor execution in terminal and RViz

---

## 📁 Project Structure

```
ur10_ws/
├── src/
│   ├── ur10e_hl_interface/           # Main ROS2 package
│   │   ├── src/
│   │   │   ├── robot_hl_interface.cpp    # High-level robot interface
│   │   │   ├── add_objects_node.cpp      # Scene object management
│   │   │   └── clipfix_bewegung.cpp      # Generated task code
│   │   ├── include/
│   │   │   └── robot_hl_interface.hpp    # Interface header
│   │   └── config/
│   │       ├── AML-Datei-V04.aml         # Component configuration
│   │       └── SchaltschrankZustand.aml  # Cabinet state (generated)
│   │
│   ├── ur_with_gripper/              # Robot description
│   └── ur_with_gripper_moveit_config/# MoveIt2 configuration
│
├── gui/                              # Python GUI application
│   ├── main.py                       # Main GUI (PyQt6)
│   ├── prompts.py                    # Prompt generation
│   ├── aml_parser.py                 # AML file parser
│   ├── aml_prompt_parser.py          # Prompt-specific parser
│   ├── InterActLLM.py                # LLM interface (OpenAI/Claude/Ollama)
│   └── resources/
│       └── LPS_logo.png
│
├── launcher.py                       # Automated ROS2 launcher
├── README.md                         # This file
```

### Key Files Explained

#### **C++ High-Level Interface**

| File | Purpose |
|------|---------|
| `robot_hl_interface.hpp/cpp` | Core robot control interface with 4 main functions:<br>• `pick(component)` - Pick and place on rail<br>• `place_rail(position)` - Move rail to cabinet<br>• `pick_rail(position)` - Get rail from storage/cabinet<br>• `detach_from_rail(component)` - Remove component from rail |
| `add_objects_node.cpp` | Loads 3D models and spawns objects in Gazebo/RViz from AML configuration |

#### **Python GUI & LLM Integration**

| File | Purpose |
|------|---------|
| `main.py` | Main GUI application (PyQt6):<br>• User interface<br>• Multi-threaded LLM requests<br>• Status monitoring<br>• Code execution |
| `prompts.py` | System prompt generation for Level 1 & 2 |
| `aml_prompt_parser.py` | Singleton parser for AML configuration and state:<br>• Component availability tracking<br>• Cabinet state management<br>• Validation logic |
| `InterActLLM.py` | Unified LLM interface supporting:<br>• OpenAI (GPT-4)<br>• Anthropic (Claude)<br>• Ollama (local models: Llama, Deepseek, etc.) |

#### **Configuration**

| File | Purpose |
|------|---------|
| `AML-Datei-V04.aml` | AutomationML configuration:<br>• Component definitions (terminals, rails)<br>• Geometric properties (width, color, size)<br>• Pick positions<br>• Robot parameters (approach height, spacing) |
| `SchaltschrankZustand.aml` | Runtime state (auto-generated):<br>• Current component locations<br>• Rail positions (storage/workspace/cabinet)<br>• Cabinet occupation |

---

## 🎓 High-Level Interface API

The system provides 4 core functions for robot control:

```cpp
// Pick component from storage and place on current rail
bool pick(const std::string& component_name);

// Move current rail (with all components) to cabinet
bool place_rail(int position = -1);  // -1 = next available position

// Get rail from storage (-1) or cabinet position (1-5)
bool pick_rail(int position = -1);

// Remove component from rail (LIFO - stack principle)
bool detach_from_rail(const std::string& component_name);
```




