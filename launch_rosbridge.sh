#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/levin/rosbridge_ws/install/setup.bash

# Launch rosbridge server and rosapi node
echo "Starting rosbridge server and rosapi..."
echo "This allows the ROS MCP server to connect to your simulated robot"

# Start rosapi_node in background (required for MCP server)
ros2 run rosapi rosapi_node &
ROSAPI_PID=$!

# Trap to kill rosapi when script exits
trap "kill $ROSAPI_PID 2>/dev/null" EXIT

# Launch rosbridge server (foreground)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
