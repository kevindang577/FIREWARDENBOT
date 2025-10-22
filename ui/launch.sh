#!/bin/bash

# Fire Warden Bot UI Launcher
# This script sets up and launches the UI with ROS 2 integration

set -e

echo "🔥 Fire Warden Bot UI Launcher 🔥"
echo "=================================="

# Check if we're in the right directory
if [ ! -f "index.html" ]; then
    echo "❌ Error: Please run this script from the ui/ directory"
    exit 1
fi

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to find an available port
find_available_port() {
    local port=8080
    while lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; do
        port=$((port + 1))
    done
    echo $port
}

# Function to start simple HTTP server
start_http_server() {
    local port=$(find_available_port)
    echo "🌐 Starting web server on port $port..."
    
    if command_exists python3; then
        echo "   Using Python 3 HTTP server"
        python3 -m http.server $port > /dev/null 2>&1 &
        SERVER_PID=$!
    elif command_exists python; then
        echo "   Using Python 2 HTTP server"
        python -m SimpleHTTPServer $port > /dev/null 2>&1 &
        SERVER_PID=$!
    elif command_exists node; then
        echo "   Using Node.js HTTP server"
        npx http-server -p $port --silent > /dev/null 2>&1 &
        SERVER_PID=$!
    else
        echo "❌ Error: No suitable HTTP server found (Python or Node.js required)"
        exit 1
    fi
    
    echo "   Server PID: $SERVER_PID"
    echo "   URL: http://localhost:$port"
    echo
}

# Function to start ROSBridge (optional)
start_rosbridge() {
    if command_exists ros2; then
        echo "🤖 Checking ROS 2 environment..."
        
        if [ -z "$ROS_DISTRO" ]; then
            echo "   Sourcing ROS 2 setup..."
            source /opt/ros/humble/setup.bash 2>/dev/null || {
                echo "   ⚠️  ROS 2 Humble not found, continuing without ROS integration"
                return 1
            }
        fi
        
        echo "   ROS Distribution: $ROS_DISTRO"
        
        # Check if rosbridge is installed
        if ros2 pkg list | grep -q rosbridge_server; then
            echo "   Starting ROSBridge WebSocket server..."
            ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &
            ROSBRIDGE_PID=$!
            echo "   ROSBridge PID: $ROSBRIDGE_PID"
            echo "   WebSocket: ws://localhost:9090"
        else
            echo "   ⚠️  ROSBridge not installed. Install with:"
            echo "      sudo apt install ros-$ROS_DISTRO-rosbridge-suite"
            echo "   Continuing without ROS integration..."
            return 1
        fi
    else
        echo "   ⚠️  ROS 2 not found, continuing without ROS integration"
        return 1
    fi
    echo
}

# Function to open browser
open_browser() {
    local url="http://localhost:$1"
    echo "🌐 Opening Fire Warden Bot UI..."
    
    # Wait a moment for server to start
    sleep 2
    
    if command_exists xdg-open; then
        xdg-open "$url" > /dev/null 2>&1
    elif command_exists open; then
        open "$url" > /dev/null 2>&1
    elif command_exists firefox; then
        firefox "$url" > /dev/null 2>&1 &
    elif command_exists google-chrome; then
        google-chrome "$url" > /dev/null 2>&1 &
    elif command_exists chromium-browser; then
        chromium-browser "$url" > /dev/null 2>&1 &
    else
        echo "   Please open your browser and navigate to: $url"
    fi
}

# Function to cleanup on exit
cleanup() {
    echo
    echo "🧹 Cleaning up..."
    
    if [ ! -z "$SERVER_PID" ]; then
        echo "   Stopping web server (PID: $SERVER_PID)"
        kill $SERVER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$ROSBRIDGE_PID" ]; then
        echo "   Stopping ROSBridge (PID: $ROSBRIDGE_PID)"
        kill $ROSBRIDGE_PID 2>/dev/null || true
    fi
    
    echo "   Goodbye! 👋"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Main execution
echo "📋 Pre-flight checks..."

# Start HTTP server
start_http_server
HTTP_PORT=$port

# Start ROSBridge (optional)
if [ "$1" != "--no-ros" ]; then
    start_rosbridge
    ROS_ENABLED=$?
else
    echo "🤖 Skipping ROS integration (--no-ros flag)"
    ROS_ENABLED=1
    echo
fi

# Open browser
open_browser $HTTP_PORT

echo "✅ Fire Warden Bot UI is ready!"
echo
echo "📊 System Status:"
echo "   • Web UI: http://localhost:$HTTP_PORT"
if [ $ROS_ENABLED -eq 0 ]; then
    echo "   • ROS Integration: ✅ Active (ws://localhost:9090)"
else
    echo "   • ROS Integration: ❌ Disabled"
fi
echo
echo "🎮 UI Features:"
echo "   • Mission Planning & Control"
echo "   • Real-time KPI Dashboard" 
echo "   • Session Recording (rosbag2)"
echo "   • Fire Detection Alerts"
echo "   • Multi-world Environment Support"
echo
echo "⌨️  Keyboard Shortcuts:"
echo "   • Ctrl+Enter  - Launch Mission"
echo "   • Ctrl+Space  - Pause Mission"
echo "   • Ctrl+H      - Return Home"
echo "   • Ctrl+X      - Emergency Stop"
echo
echo "🏞️  Available Worlds:"
echo "   • Simple Trees    • Dense Forest"
echo "   • Sparse Forest   • Mixed Terrain"
echo "   • Open Meadows    • Obstacle Course"
echo "   • Large Demo"
echo
echo "📖 For more information, see README.md"
echo
echo "Press Ctrl+C to stop all services and exit"

# Keep script running until interrupted
while true; do
    sleep 1
done
