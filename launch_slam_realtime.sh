#!/bin/bash

# Fire Warden Bot SLAM Real-Time Mapping Launcher
# Complete system launch for live SLAM visualization

set -e

echo "🗺️ Fire Warden Bot - Real-Time SLAM Mapping System 🗺️"
echo "========================================================="

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Check if workspace is built
if [ -d "install" ]; then
    source install/setup.bash
    echo "✅ Using built workspace"
else
    echo "⚠️  Building workspace first..."
    colcon build --symlink-install
    source install/setup.bash
fi``

echo ""
echo "🚀 Launching Complete SLAM Mapping System..."
echo ""

# Function to check if a process is running
check_process() {
    if pgrep -f "$1" > /dev/null; then
        echo "✅ $1 is running"
        return 0
    else
        echo "❌ $1 is not running"
        return 1
    fi
}

# Kill any existing Gazebo/Ignition processes
echo "🧹 Cleaning up existing processes..."
pkill -f "ign gazebo" || true
pkill -f "gz sim" || true
pkill -f "gzserver" || true
sleep 2

echo ""
echo "📡 Step 1: Launching Simulation with Drone..."
ros2 launch bringup sim_one_drone.launch.py world:=box_arena.sdf &
SIM_PID=$!
echo "   Simulation PID: $SIM_PID"

# Wait for simulation to start
echo "   ⏳ Waiting for simulation to initialize..."
sleep 8

# Check if simulation topics are available
echo "   🔍 Checking simulation topics..."
timeout 10 bash -c 'until ros2 topic list | grep -q "/model/drone1/scan"; do sleep 1; done' || {
    echo "   ❌ Simulation failed to start properly"
    exit 1
}
echo "   ✅ Simulation is ready"

echo ""
echo "🗺️ Step 2: Launching SLAM Mapping..."
ros2 launch bringup slam_mapping.launch.py drone_name:=drone1 mapping_mode:=true &
SLAM_PID=$!
echo "   SLAM PID: $SLAM_PID"

# Wait for SLAM to initialize
echo "   ⏳ Waiting for SLAM to initialize..."
sleep 5

echo ""
echo "👁️ Step 3: Launching RViz for Real-Time Visualization..."
ros2 run rviz2 rviz2 -d /home/student/git/FIREWARDENBOT/src/bringup/config/slam_visualization.rviz &
RVIZ_PID=$!
echo "   RViz PID: $RVIZ_PID"

echo ""
echo "🎮 Step 4: Launching Teleoperation Control..."
echo "   Starting keyboard teleop in new terminal..."
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; echo 'Use WASD keys to drive the drone and build the map!'; echo 'Press Ctrl+C in this terminal to stop teleop'; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/drone1/cmd_vel; exec bash" &
TELEOP_PID=$!

sleep 3

echo ""
echo "✅ All Systems Launched Successfully!"
echo ""
echo "🎯 Real-Time SLAM Mapping is now active:"
echo "   📊 Map Topic: /map"
echo "   📍 Robot Pose: /odom"
echo "   🚁 Drone Control: Use WASD in teleop terminal"
echo "   👁️ Visualization: RViz window should be open"
echo ""

echo "🗺️ Map Information:"
echo "   • Real-time map: Available on /map topic"
echo "   • Map updates: Every 2 seconds (configurable)"
echo "   • Map resolution: 0.05m per pixel"
echo "   • Map frame: 'map'"
echo ""

echo "💾 Saving Maps:"
echo "   • Auto-save location: /tmp/firewardenbot_map.*"
echo "   • Manual save: ros2 run nav2_map_server map_saver_cli -f my_map"
echo "   • View saved map: eog /tmp/firewardenbot_map.pgm"
echo ""

echo "🌐 Web Interface:"
echo "   • Launch: cd ui && ./launch.sh"
echo "   • SLAM API: http://localhost:8090/api/slam"
echo "   • Map data: http://localhost:8090/api/mapping"
echo ""

echo "🎮 Controls:"
echo "   • Drive drone: Use WASD keys in teleop terminal"
echo "   • View map: Check RViz 'Map' display"
echo "   • Monitor: ros2 topic echo /map --once"
echo "   • Save map: Ctrl+C here, then run save command"
echo ""

# Function to handle cleanup
cleanup() {
    echo ""
    echo "🛑 Shutting down SLAM Mapping System..."
    
    # Kill processes in reverse order
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
    
    # Clean up any remaining processes
    pkill -f "ign gazebo" || true
    pkill -f "slam_toolbox" || true
    
    echo ""
    echo "💾 Saving final map..."
    timeout 10 ros2 run nav2_map_server map_saver_cli -f /tmp/firewardenbot_final_map || echo "   ⚠️ Could not save final map"
    
    echo ""
    echo "📊 Session Summary:"
    if [ -f "/tmp/firewardenbot_final_map.yaml" ]; then
        echo "   ✅ Map saved: /tmp/firewardenbot_final_map.*"
        echo "   📁 View with: eog /tmp/firewardenbot_final_map.pgm"
    fi
    
    echo "🗺️ SLAM Mapping Session Complete!"
    exit 0
}

# Set up signal handling
trap cleanup SIGINT SIGTERM

echo "🔄 System Status Monitor:"
echo "   Press Ctrl+C to stop and save the map"
echo ""

# Monitor system status
while true; do
    sleep 30
    
    echo "⏰ Status ($(date '+%H:%M:%S')):"
    
    # Check if SLAM is publishing maps
    if timeout 5 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   🟢 SLAM mapping active"
    else
        echo "   🟡 SLAM mapping not detected"
    fi
    
    # Check if robot is moving
    if timeout 5 ros2 topic echo /model/drone1/cmd_vel --once > /dev/null 2>&1; then
        echo "   🟢 Robot control active"
    else
        echo "   🟡 Robot not moving"
    fi
    
    echo "   💡 Drive the robot around to build a complete map!"
done
