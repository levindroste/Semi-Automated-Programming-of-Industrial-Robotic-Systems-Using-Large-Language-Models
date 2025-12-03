# ROS MCP Server Setup

This document describes the ROS MCP server setup for this project.

## What's Installed

### 1. rosbridge_server
- **Location**: `~/rosbridge_ws`
- **Built from source** (ROS 2 Humble)
- **Purpose**: Provides WebSocket interface for ROS communication

### 2. ros-mcp-server
- **Location**: `~/ros-mcp-server`
- **Purpose**: MCP server that connects Claude Code to ROS via rosbridge

### 3. MCP Configuration
- **File**: `.mcp.json` (in this directory)
- **Purpose**: Automatically loads the ROS MCP server when working in this folder

## How to Use

### Step 1: Start rosbridge

Before using ROS commands with Claude, you must start rosbridge in a terminal:

```bash
./start_rosbridge.sh
```

Or manually:
```bash
source /opt/ros/humble/setup.bash
source ~/rosbridge_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Keep this terminal running** while you work with Claude.

### Step 2: Reload Claude Code

After starting rosbridge, reload your Claude Code workspace (or restart VSCode) to ensure the MCP server connects.

### Step 3: Verify MCP Server is Loaded

Type `/mcp` in Claude Code to see if the ros-mcp-server is listed and connected.

### Step 4: Use ROS Commands

You can now ask Claude to:
- List ROS topics: "What topics are available?"
- Query robot state: "Show me the robot description"
- Control robots: "Move the robot forward"
- Monitor sensors: "What's the current camera feed?"

## Testing the Setup

### Test with turtlesim (Optional)

In a new terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

Then ask Claude: "Move the turtle forward"

## Troubleshooting

### MCP server not showing in `/mcp`
1. Make sure rosbridge is running
2. Reload the Claude Code workspace
3. Check `.mcp.json` configuration

### "Connection refused" errors
- Ensure rosbridge is running (`./start_rosbridge.sh`)
- Check that port 9090 is not blocked

### Build errors with rosbridge
- The CMakeLists.txt files were modified to make `ament_cmake_mypy` optional
- If you need to rebuild: `cd ~/rosbridge_ws && colcon build --packages-select rosbridge_suite`

## Architecture

```
Claude Code → ros-mcp-server → rosbridge (WebSocket) → ROS 2 Humble
```

- **Claude Code**: Your AI assistant interface
- **ros-mcp-server**: Translates MCP protocol to rosbridge calls
- **rosbridge**: Translates WebSocket/HTTP to ROS messages
- **ROS 2**: Your robot system

## Files Modified

- `~/rosbridge_ws/src/rosbridge_suite/*/CMakeLists.txt` - Made mypy optional for building
