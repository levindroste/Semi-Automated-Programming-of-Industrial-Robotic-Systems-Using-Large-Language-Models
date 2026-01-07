#!/bin/bash
# Helper script to start rosbridge for the ROS MCP server

echo "Starting rosbridge WebSocket server..."
echo "This must be running for the ROS MCP server to work."
echo "Press Ctrl+C to stop."
echo ""

source /opt/ros/humble/setup.bash
source ~/rosbridge_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
