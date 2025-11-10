#!/bin/bash

# Fire Warden Bot - Simple Working SLAM Launch
# This version avoids ROS daemon issues and launches step by step

echo "🗺️ Fire Warden Bot - Simple SLAM Mapping 🗺️"
echo "============================================="

# Kill everything first
echo "🧹 Cleaning up..."
pkill -f "ros2\|slam\|gazebo\|ign" 2>/dev/null || true
sleep 3

# Source ROS
echo "🔧 Setting up ROS environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ""
echo "🚀 Step 1: Starting simulation..."
echo "This will take about 10 seconds to fully initialize..."

# Start simulation in background
ros2 launch bringup sim_one_drone.launch.py world:=box_arena.sdf &
SIM_PID=$!
echo "Simulation PID: $SIM_PID"

# Wait for simulation
sleep 12
echo "✅ Simulation should be ready"

echo ""
echo "🗺️ Step 2: Starting SLAM mapping..."

# Start SLAM in background  
ros2 launch bringup slam_mapping.launch.py drone_name:=drone1 mapping_mode:=true &
SLAM_PID=$!
echo "SLAM PID: $SLAM_PID"

# Wait for SLAM
sleep 6
echo "✅ SLAM should be ready"

echo ""
echo "👁️ Step 3: Starting RViz..."

# Start RViz
ros2 run rviz2 rviz2 &
RVIZ_PID=$!
echo "RViz PID: $RVIZ_PID"

echo ""
echo "🎮 Step 4: Robot control..."
echo "In RViz:"
echo "1. Add -> By display type -> Map"
echo "2. Set topic to: /map"
echo "3. Add -> By display type -> LaserScan" 
echo "4. Set topic to: /model/drone1/scan"
echo "5. Set Fixed Frame to: map"

echo ""
echo "To control the robot, run in a NEW TERMINAL:"
echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/drone1/cmd_vel"

echo ""
echo "💾 To save the map when done:"
echo "ros2 run nav2_map_server map_saver_cli -f my_map"

echo ""
echo "📊 Monitor progress:"
echo "The map will appear in RViz as you drive the robot around!"

# Cleanup function
cleanup() {
    echo ""
    echo "🛑 Shutting down..."
    if [ ! -z "$RVIZ_PID" ]; then kill $RVIZ_PID 2>/dev/null || true; fi
    if [ ! -z "$SLAM_PID" ]; then kill $SLAM_PID 2>/dev/null || true; fi  
    if [ ! -z "$SIM_PID" ]; then kill $SIM_PID 2>/dev/null || true; fi
    pkill -f "ros2\|slam\|gazebo\|ign" 2>/dev/null || true
    echo "🗺️ SLAM system stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo ""
echo "✅ All systems started!"
echo "🔄 Press Ctrl+C to stop everything"
echo ""
echo "Next steps:"
echo "1. Wait for RViz to open"
echo "2. Configure RViz displays (see above)"
echo "3. Open new terminal and run teleop command"
echo "4. Drive robot with WASD keys to build map!"

# Keep running
while true; do
    sleep 60
    echo "⏰ System running... ($(date '+%H:%M:%S'))"
    echo "   Drive the robot to build the map!"
done
