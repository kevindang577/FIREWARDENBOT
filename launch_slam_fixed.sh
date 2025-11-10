#!/bin/bash

# Fire Warden Bot SLAM - Fixed Launch
# Properly waits for each component

echo "🗺️ Fire Warden Bot - SLAM System 🗺️"
echo "===================================="

# Clean start
echo "🧹 Cleaning up..."
pkill -f "ros2\|slam\|gazebo\|ign\|rviz" 2>/dev/null || true
sleep 3

# Source ROS
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ""
echo "🚀 Starting simulation ONLY first..."

# Start simulation
ros2 launch bringup sim_one_drone.launch.py world:=box_arena.sdf > /tmp/sim.log 2>&1 &
SIM_PID=$!

echo "Simulation starting (PID: $SIM_PID)..."
echo "Waiting for scan topic..."

# Wait more patiently for scan topic
for i in {1..60}; do
    if timeout 2 ros2 topic list 2>/dev/null | grep -q "/model/drone1/scan"; then
        echo "✅ Scan topic found after ${i}s"
        break
    fi
    if [ $i -eq 60 ]; then
        echo "❌ Scan topic never appeared"
        echo "Last simulation log:"
        tail -5 /tmp/sim.log
        exit 1
    fi
    sleep 1
    if [ $((i % 10)) -eq 0 ]; then
        echo "   Still waiting... (${i}/60s)"
    fi
done

echo ""
echo "🗺️ Now starting SLAM..."

# Start SLAM 
ros2 launch bringup slam_mapping.launch.py drone_name:=drone1 mapping_mode:=true > /tmp/slam.log 2>&1 &
SLAM_PID=$!

echo "SLAM starting (PID: $SLAM_PID)..."
echo "Waiting for map topic..."

# Wait for map topic
for i in {1..30}; do
    if timeout 2 ros2 topic list 2>/dev/null | grep -q "/map"; then
        echo "✅ Map topic found after ${i}s"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "❌ Map topic never appeared"
        echo "SLAM log:"
        tail -10 /tmp/slam.log
        exit 1
    fi
    sleep 1
    if [ $((i % 5)) -eq 0 ]; then
        echo "   Still waiting... (${i}/30s)"
    fi
done

echo ""
echo "✅ SLAM System Ready!"
echo ""
echo "📊 Active Topics:"
timeout 3 ros2 topic list 2>/dev/null | grep -E "(map|scan|odom|cmd_vel)" | sort

echo ""
echo "🎮 TO DRIVE THE ROBOT:"
echo "Open a NEW terminal and run:"
echo "   cd /home/student/git/FIREWARDENBOT"
echo "   source /opt/ros/humble/setup.bash && source install/setup.bash"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/drone1/cmd_vel"

echo ""
echo "👁️ TO VIEW THE MAP:"
echo "The map is being published on /map topic"
echo "You can view it with: ros2 topic echo /map --once"

echo ""
echo "💾 TO SAVE THE MAP:"
echo "After driving around to build the map, type 'save' and press Enter"

# Save function
save_current_map() {
    local map_name=${1:-"firewardenbot_$(date +%Y%m%d_%H%M%S)"}
    echo ""
    echo "💾 Saving map as: $map_name"
    
    # Check if map topic has data first
    echo "🔍 Checking if map has data..."
    if timeout 10 ros2 topic echo /map -1 > /tmp/map_check.txt 2>&1; then
        map_size=$(grep -c "data:" /tmp/map_check.txt 2>/dev/null || echo "0")
        if [ "$map_size" -gt 0 ]; then
            echo "   ✅ Map has data (${map_size} cells)"
            
            # Save the map
            if ros2 run nav2_map_server map_saver_cli -f "$map_name"; then
                echo "✅ Map saved successfully!"
                ls -la "${map_name}".* 2>/dev/null | sed 's/^/   📁 /'
            else
                echo "❌ Map save failed"
            fi
        else
            echo "   ⚠️ Map appears empty - drive robot around first!"
        fi
    else
        echo "   ❌ Cannot read map topic"
    fi
    rm -f /tmp/map_check.txt
}

# Interactive loop
while true; do
    echo ""
    read -p "Command (save/check/quit): " cmd
    
    case "$cmd" in
        save)
            save_current_map
            ;;
        save\ *)
            map_name=$(echo "$cmd" | cut -d' ' -f2)
            save_current_map "$map_name"
            ;;
        check)
            echo "📊 System Status:"
            echo "Active nodes:"
            timeout 3 ros2 node list 2>/dev/null | sed 's/^/   • /' || echo "   ❌ Cannot list nodes"
            echo "Key topics:"
            timeout 3 ros2 topic list 2>/dev/null | grep -E "(map|scan|cmd_vel)" | sed 's/^/   • /' || echo "   ❌ Cannot list topics"
            ;;
        quit|exit|q)
            break
            ;;
        *)
            echo "Commands: save [name], check, quit"
            ;;
    esac
done

# Cleanup
echo ""
echo "🛑 Stopping SLAM system..."
[ ! -z "$SLAM_PID" ] && kill $SLAM_PID 2>/dev/null
[ ! -z "$SIM_PID" ] && kill $SIM_PID 2>/dev/null
pkill -f "ros2\|slam\|gazebo\|ign" 2>/dev/null || true
echo "✅ Cleanup complete"
