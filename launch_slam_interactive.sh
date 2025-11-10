#!/bin/bash

# Fire Warden Bot SLAM Quick Start & Map Saver
# Starts SLAM system and saves maps properly

echo "🗺️ Fire Warden Bot - SLAM Quick Start 🗺️"
echo "========================================="

# Clean start
echo "🧹 Cleaning up any existing processes..."
pkill -f "ros2\|slam\|gazebo\|ign" 2>/dev/null || true
sleep 2

# Source ROS
echo "🔧 Setting up ROS environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ""
echo "🚀 Starting SLAM system..."

# Function to wait for topic
wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    echo "   ⏳ Waiting for topic $topic..."
    
    local count=0
    while [ $count -lt $timeout ]; do
        if timeout 3 ros2 topic list 2>/dev/null | grep -q "$topic"; then
            echo "   ✅ Topic $topic is available"
            return 0
        fi
        sleep 1
        count=$((count + 1))
        if [ $((count % 10)) -eq 0 ]; then
            echo "      Still waiting... ($count/${timeout}s)"
        fi
    done
    
    echo "   ❌ Topic $topic not available after ${timeout}s" 
    return 1
}

# Start simulation
echo "📡 Step 1: Starting simulation..."
ros2 launch bringup sim_one_drone.launch.py world:=box_arena.sdf > /tmp/sim.log 2>&1 &
SIM_PID=$!
echo "   Simulation PID: $SIM_PID"

# Wait for simulation to be ready
if ! wait_for_topic "/model/drone1/scan" 45; then
    echo "❌ Simulation failed to start properly"
    echo "📄 Simulation log:"
    tail -10 /tmp/sim.log
    exit 1
fi

# Start SLAM
echo ""
echo "🗺️ Step 2: Starting SLAM mapping..."
ros2 launch bringup slam_mapping.launch.py drone_name:=drone1 mapping_mode:=true > /tmp/slam.log 2>&1 &
SLAM_PID=$!
echo "   SLAM PID: $SLAM_PID"

# Wait for SLAM to be ready
if ! wait_for_topic "/map" 30; then
    echo "❌ SLAM failed to start properly"
    echo "📄 SLAM log:"
    tail -10 /tmp/slam.log
    exit 1
fi

echo ""
echo "✅ SLAM System is ready!"
echo ""

# Check what we have
echo "📊 System Status:"
echo "Available topics:"
timeout 5 ros2 topic list 2>/dev/null | grep -E "(map|scan|odom)" | sed 's/^/   • /' || echo "   ⚠️ Could not list topics"

echo ""
echo "ROS nodes:"
timeout 5 ros2 node list 2>/dev/null | sed 's/^/   • /' || echo "   ⚠️ Could not list nodes"

# Function to save map
save_map() {
    local map_name=$1
    echo ""
    echo "💾 Saving map as: $map_name"
    
    # Try to save map with better error handling
    if timeout 15 ros2 run nav2_map_server map_saver_cli -f "$map_name"; then
        echo "✅ Map saved successfully!"
        if [ -f "${map_name}.yaml" ]; then
            echo "📁 Files created:"
            ls -la "${map_name}".* | sed 's/^/   • /'
            echo "👁️ View map: eog ${map_name}.pgm"
        fi
    else
        echo "❌ Map save failed. Troubleshooting..."
        
        # Check if map topic has data
        echo "🔍 Checking map topic..."
        if timeout 5 ros2 topic echo /map --once > /dev/null 2>&1; then
            echo "   ✅ Map topic has data"
            # Try alternative save method
            echo "   🔄 Trying alternative save method..."
            timeout 10 ros2 topic pub /map_save std_msgs/msg/String "data: '$map_name'" --once || true
        else
            echo "   ❌ Map topic has no data"
            echo "   Possible issues:"
            echo "   • SLAM not generating map yet (drive robot around)"
            echo "   • SLAM configuration issue"
            echo "   • Topic remapping problem"
        fi
    fi
}

# Start RViz for visualization
echo ""
echo "👁️ Starting RViz..."
ros2 run rviz2 rviz2 > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
echo "   RViz PID: $RVIZ_PID"

echo ""
echo "🎮 Robot Control:"
echo "Open a NEW terminal and run:"
echo "   source /opt/ros/humble/setup.bash && source install/setup.bash"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/drone1/cmd_vel"

echo ""
echo "🗺️ Map Building:"
echo "1. Drive robot around with WASD keys"
echo "2. Watch map build in RViz (add Map display, topic: /map)"
echo "3. Save map with this script's commands"

echo ""
echo "Available commands:"
echo "   save [name] - Save current map"
echo "   status      - Check system status"
echo "   quit        - Stop everything"

# Interactive command loop
while true; do
    echo ""
    read -p "🎯 Enter command (save/status/quit): " cmd
    
    case "$cmd" in
        save)
            read -p "Map name (default: firewardenbot_map): " map_name
            map_name=${map_name:-firewardenbot_map}
            save_map "$map_name"
            ;;
        save\ *)
            map_name=$(echo "$cmd" | cut -d' ' -f2-)
            save_map "$map_name"
            ;;
        status)
            echo "📊 System Status:"
            echo "Processes:"
            ps aux | grep -E "(slam|gazebo)" | grep -v grep | sed 's/^/   • /'
            echo "Topics:"
            timeout 3 ros2 topic list 2>/dev/null | grep -E "(map|scan|odom)" | sed 's/^/   • /' || echo "   ⚠️ Could not list topics"
            ;;
        quit|exit|q)
            break
            ;;
        *)
            echo "❓ Unknown command. Use: save, status, or quit"
            ;;
    esac
done

# Cleanup
echo ""
echo "🛑 Shutting down SLAM system..."

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

pkill -f "ros2\|slam\|gazebo\|ign" 2>/dev/null || true

echo "🗺️ SLAM session complete!"
echo ""
echo "📁 Check for saved maps:"
echo "   ls -la *firewardenbot*.* *.yaml *.pgm"
