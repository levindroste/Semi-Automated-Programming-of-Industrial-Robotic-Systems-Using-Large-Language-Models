# ROS MCP Server Setup Guide

## ✅ Installation Complete!

The ROS MCP server has been installed and configured for use with Claude Code in VS Code.

## 🚀 How to Use

### Step 1: Start Rosbridge Server
Before using the MCP server, you need to start the rosbridge server which allows communication with ROS:

```bash
./launch_rosbridge.sh
```

This script:
- Sources your ROS2 Humble environment
- Sources your rosbridge workspace (built from Humble-compatible branch)
- Launches `rosapi_node` (required for MCP server to query topics/services)
- Launches `rosbridge_websocket` on port 9090

**Keep this terminal running** - rosbridge needs to be active for the MCP server to work.

### Step 2: Restart Claude Code
After starting rosbridge, restart your Claude Code window to initialize the MCP server connection:
- Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
- Type "Developer: Reload Window"
- Press Enter

### Step 3: Verify MCP Connection
Check if the MCP server is connected by typing:
```
/mcp
```

You should see `ros-mcp-server` listed with available tools like:
- `ros_list_topics` - List all ROS topics
- `ros_list_services` - List all ROS services
- `ros_subscribe_topic` - Subscribe to topics
- `ros_publish_topic` - Publish to topics
- `ros_call_service` - Call ROS services
- And more...

### Step 4: Start Your Simulation
Launch your UR10e simulation in Gazebo:
```bash
source /opt/ros/humble/setup.bash
# Your usual launch command for UR10e + MoveIt
```

### Step 5: Control Your Robot with Natural Language
Now you can interact with your simulated robot using natural language! For example:
- "What ROS topics are available?"
- "Subscribe to the joint_states topic"
- "What services does the robot provide?"
- "Call the /get_planning_scene service"

## 📁 Configuration Files

- [.mcp.json](.mcp.json) - MCP server configuration for Claude Code
- [launch_rosbridge.sh](launch_rosbridge.sh) - Convenience script to start rosbridge

## 🔧 How It Works

```
┌─────────────────┐
│  Claude Code    │ ← You interact with me
│   (VS Code)     │
└────────┬────────┘
         │
         │ MCP Protocol
         │
┌────────▼────────┐
│  ros-mcp-server │ ← Translates between AI and ROS
└────────┬────────┘
         │
         │ WebSocket (port 9090)
         │
┌────────▼────────┐
│  rosbridge      │ ← Bridge between WebSocket and ROS
└────────┬────────┘
         │
         │ ROS2 Topics/Services
         │
┌────────▼────────┐
│  Your Robot     │ ← UR10e simulation in Gazebo
│  (Gazebo/RViz)  │
└─────────────────┘
```

## 🐛 Troubleshooting

### MCP Server Not Showing Up
1. Make sure rosbridge is running (`./launch_rosbridge.sh`)
2. Restart Claude Code window (Developer: Reload Window)
3. Check MCP status with `/mcp` command

### Can't Connect to Robot
1. Verify rosbridge is running: `ros2 topic list | grep rosbridge`
2. Check if your simulation is running
3. Make sure you've sourced both ROS2 and your workspace

### Connection Issues
- The MCP server connects to `localhost:9090` by default
- Make sure no firewall is blocking port 9090
- Verify rosbridge_websocket started successfully in the launch output

### "get_topics" or "get_nodes" Returns Errors
This usually means `rosapi_node` is not running. The `launch_rosbridge.sh` script should start both rosbridge AND rosapi. If rosapi fails to start:
1. Make sure you're using the Humble-compatible branch of rosbridge_suite
2. Rebuild: `cd ~/rosbridge_ws && rm -rf build install log && colcon build --symlink-install`
3. Verify rosapi is running: `ros2 node list | grep rosapi`

## 📚 Learn More

- [ROS MCP Server Documentation](https://github.com/robotmcp/ros-mcp-server)
- [Claude Code MCP Guide](https://code.claude.com/docs/en/mcp.md)
- [Rosbridge Protocol](http://wiki.ros.org/rosbridge_protocol)

## 🎯 Next Steps

1. Start rosbridge: `./launch_rosbridge.sh`
2. Reload Claude Code window
3. Launch your UR10e simulation
4. Ask me to list available ROS topics!
