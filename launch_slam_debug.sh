#!/bin/bash

# Fire Warden Bot SLAM Debug & Launch Script
# Improved version with better error handling and debugging

set -e

echo "🗺️ Fire Warden Bot - SLAM Mapping System (Debug Mode) 🗺️"
echo "============================================================"

# Source ROS 2 setup
echo "🔧 Setting up ROS 2 environment..."
source /opt/ros/humble/setup.bash

# Check and build workspace if needed
echo "🔨 Checking workspace..."
if [ -d "install" ]; then
    source install/setup.bash
    echo "✅ Using existing workspace build"
else
    echo "⚠️  No install directory found. Building workspace..."
    colcon build --symlink-install
    source install/setup.bash
fi

# Rebuild if launch files are newer than install
if [ "src/bringup/launch/slam_mapping.launch.py" -nt "install" ]; then
    echo "🔄 Launch files updated, rebuilding..."
    colcon build --packages-select bringup --symlink-install
    source install/setup.bash
fi

echo ""
echo "🔍 Pre-flight Checks..."

# Check required packages
echo "📦 Checking required packages..."
REQUIRED_PACKAGES=("slam_toolbox" "nav2_map_server" "teleop_twist_keyboard" "rviz2")
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo "   ✅ $pkg found"
    else
        echo "   ❌ $pkg missing - installing..."
        sudo apt update && sudo apt install -y ros-humble-$pkg || echo "   ⚠️ Could not install $pkg"
    fi
done

# Kill any existing processes
echo ""
echo "🧹 Cleaning up existing processes..."
pkill -f "ign gazebo" || true
pkill -f "gz sim" || true  
pkill -f "gzserver" || true
pkill -f "slam_toolbox" || true
sleep 3

# Function to wait for topic
wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    echo "   ⏳ Waiting for topic $topic (timeout: ${timeout}s)..."
    
    if timeout $timeout bash -c "until ros2 topic list | grep -q '$topic'; do sleep 1; done"; then
        echo "   ✅ Topic $topic is available"
        return 0
    else
        echo "   ❌ Topic $topic not available after ${timeout}s"
        return 1
    fi
}

# Function to check node
check_node() {
    local node_name=$1
    if ros2 node list | grep -q "$node_name"; then
        echo "   ✅ Node $node_name is running"
        return 0
    else
        echo "   ❌ Node $node_name is not running"
        return 1
    fi
}

echo ""
echo "🚀 Step 1: Launching Simulation..."
echo "Command: ros2 launch bringup sim_one_drone.launch.py world:=box_arena.sdf"

# Start simulation in background with output capture
ros2 launch bringup sim_one_drone.launch.py world:=box_arena.sdf > /tmp/sim_output.log 2>&1 &
SIM_PID=$!
echo "   Simulation started with PID: $SIM_PID"
echo "   Log file: /tmp/sim_output.log"

# Wait for simulation topics
if wait_for_topic "/model/drone1/scan" 30; then
    echo "   ✅ Simulation ready"
else
    echo "   ❌ Simulation failed to provide required topics"
    echo "   📄 Simulation log:"
    tail -20 /tmp/sim_output.log
    exit 1
fi

# Verify essential topics
echo "   🔍 Verifying simulation topics..."
REQUIRED_TOPICS=("/model/drone1/scan" "/model/drone1/odometry" "/model/drone1/cmd_vel")
for topic in "${REQUIRED_TOPICS[@]}"; do
    if ros2 topic list | grep -q "$topic"; then
        echo "   ✅ $topic available"
    else
        echo "   ❌ $topic missing"
    fi
done

echo ""
echo "🗺️ Step 2: Launching SLAM Mapping..."
echo "Command: ros2 launch bringup slam_mapping.launch.py drone_name:=drone1 mapping_mode:=true"

# Start SLAM with output capture
ros2 launch bringup slam_mapping.launch.py drone_name:=drone1 mapping_mode:=true > /tmp/slam_output.log 2>&1 &
SLAM_PID=$!
echo "   SLAM started with PID: $SLAM_PID"
echo "   Log file: /tmp/slam_output.log"

# Wait for SLAM to initialize
sleep 8

# Check if SLAM is working
if wait_for_topic "/map" 20; then
    echo "   ✅ SLAM mapping active"
else
    echo "   ❌ SLAM failed to start properly"
    echo "   📄 SLAM log:"
    tail -20 /tmp/slam_output.log
    echo ""
    echo "   🔧 Troubleshooting:"
    echo "   • Check if slam_toolbox is installed: ros2 pkg list | grep slam_toolbox"
    echo "   • Check topics: ros2 topic list | grep -E '(scan|odom|map)'"
    echo "   • Check nodes: ros2 node list"
    
    # Continue anyway for debugging
fi

echo ""
echo "👁️ Step 3: Launching RViz Visualization..."

# Check if RViz config exists
RVIZ_CONFIG="/home/student/git/FIREWARDENBOT/src/bringup/config/slam_visualization.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "   ⚠️ Custom RViz config not found, using default"
    RVIZ_CONFIG=""
else
    echo "   ✅ Using custom SLAM visualization config"
fi

# Start RViz
if [ -n "$RVIZ_CONFIG" ]; then
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz_output.log 2>&1 &
else
    ros2 run rviz2 rviz2 > /tmp/rviz_output.log 2>&1 &
fi
RVIZ_PID=$!
echo "   RViz started with PID: $RVIZ_PID"

echo ""
echo "🎮 Step 4: Setting up Robot Control..."

# Start teleop in a way that works in different environments
echo "   Starting teleop control..."
if command -v gnome-terminal > /dev/null; then
    gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash 2>/dev/null || true; echo '🎮 Robot Control Active'; echo 'Use WASD keys to drive the drone!'; echo 'Press Ctrl+C to stop teleop'; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/drone1/cmd_vel; exec bash" &
    TELEOP_PID=$!
    echo "   ✅ Teleop started in new terminal"
elif command -v xterm > /dev/null; then
    xterm -e "source /opt/ros/humble/setup.bash; source install/setup.bash 2>/dev/null || true; echo '🎮 Robot Control Active'; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/drone1/cmd_vel" &
    TELEOP_PID=$!
    echo "   ✅ Teleop started in xterm"
else
    echo "   ⚠️ No terminal emulator found. Start teleop manually:"
    echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/drone1/cmd_vel"
    TELEOP_PID=""
fi

sleep 3

echo ""
echo "📊 System Status Check..."
echo "Current ROS nodes:"
ros2 node list | sed 's/^/   • /'

echo ""
echo "Current ROS topics:"
ros2 topic list | grep -E "(map|scan|odom|cmd_vel)" | sed 's/^/   • /' || echo "   ⚠️ No relevant topics found"

echo ""
echo "✅ Launch Complete! System Status:"

# Final status check
if ros2 topic list | grep -q "/map"; then
    echo "   🟢 SLAM mapping: ACTIVE"
else
    echo "   🔴 SLAM mapping: NOT DETECTED"
    echo "      Possible issues:"
    echo "      • slam_toolbox not installed: sudo apt install ros-humble-slam-toolbox"
    echo "      • Launch file issues: check /tmp/slam_output.log"
    echo "      • Topic remapping problems"
fi

if ros2 topic list | grep -q "/model/drone1/scan"; then
    echo "   🟢 Simulation: ACTIVE"
else
    echo "   🔴 Simulation: NOT DETECTED"
fi

if ros2 node list | grep -q "teleop"; then
    echo "   🟢 Robot control: ACTIVE"
else
    echo "   🟡 Robot control: Start manually if needed"
fi

echo ""
echo "🎯 Usage Instructions:"
echo "   1. 🎮 Drive robot: Use WASD keys in teleop window"
echo "   2. 👁️ View map: Check RViz window for real-time map"
echo "   3. 📊 Monitor: ros2 topic echo /map --field info"
echo "   4. 💾 Save map: ros2 run nav2_map_server map_saver_cli -f my_map"
echo ""

echo "🔧 Debugging Commands:"
echo "   • Check SLAM: ros2 topic echo /map --once"
echo "   • Monitor laser: ros2 topic echo /model/drone1/scan --field header"
echo "   • View logs: tail -f /tmp/slam_output.log"
echo "   • List nodes: ros2 node list"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "🛑 Shutting down SLAM system..."
    
    if [ ! -z "$TELEOP_PID" ]; then
        kill $TELEOP_PID 2>/dev/null || true
        echo "   ✅ Teleop stopped"
    fi
    
    if [ ! -z "$RVIZ_PID" ]; then
        kill $RVIZ_PID 2>/dev/null || true
        echo "   ✅ RViz stopped"
    fi
    
    if [ ! -z "$SLAM_PID" ]; then
        kill $SLAM_PID 2>/dev/null || true
        echo "   ✅ SLAM stopped"
    fi
    
    if [ ! -z "$SIM_PID" ]; then
        kill $SIM_PID 2>/dev/null || true
        echo "   ✅ Simulation stopped"
    fi
    
    # Save map if possible
    echo "   💾 Attempting to save map..."
    timeout 10 ros2 run nav2_map_server map_saver_cli -f /tmp/firewardenbot_session_map 2>/dev/null || echo "   ⚠️ Could not save map"
    
    echo "🗺️ SLAM session complete!"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "🔄 System Monitor Active (Press Ctrl+C to stop):"
while true; do
    sleep 30
    echo "⏰ Status ($(date '+%H:%M:%S')):"
    
    if timeout 3 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   🟢 SLAM mapping: Active"
    else
        echo "   🟡 SLAM mapping: Not detected"
    fi
    
    if ros2 node list | grep -q teleop; then
        echo "   🟢 Robot control: Active"
    else
        echo "   🟡 Robot control: Not active"
    fi
    
    echo "   💡 Drive the robot around to build the map!"
done
