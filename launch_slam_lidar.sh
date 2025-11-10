#!/bin/bash

# Enhanced LiDAR Processing Launcher for Fire Warden Bot
# Integrates SLAM mapping with advanced LiDAR processing

set -e

echo "🔥 Fire Warden Bot - Enhanced LiDAR & SLAM System 🔥"
echo "======================================================="

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Check if workspace is built
if [ -d "install" ]; then
    source install/setup.bash
    echo "✅ Using built workspace"
else
    echo "⚠️  Workspace not built - some features may not work"
fi

# Set Python path to include our packages
export PYTHONPATH="/home/student/git/FIREWARDENBOT/src/vision:$PYTHONPATH"

echo ""
echo "🚀 Launching Enhanced LiDAR & SLAM System..."
echo "This system provides:"
echo "  • Real-time SLAM mapping with LiDAR"
echo "  • Advanced obstacle detection"
echo "  • Point cloud generation"
echo "  • Scan quality analysis"
echo "  • Integration with Fire Warden Bot navigation"
echo ""

# Function to check if a ROS node is running
check_node() {
    if ros2 node list | grep -q "$1"; then
        echo "✅ $1 is running"
        return 0
    else
        echo "❌ $1 is not running"
        return 1
    fi
}

# Function to check if a topic has data
check_topic() {
    if timeout 5 ros2 topic echo "$1" --once > /dev/null 2>&1; then
        echo "✅ Topic $1 has data"
        return 0
    else
        echo "⚠️  Topic $1 has no data or doesn't exist"
        return 1
    fi
}

echo "🔍 System Check..."

# Check if simulation is running
if ! check_topic "/model/drone1/scan"; then
    echo ""
    echo "❓ LiDAR simulation not detected. To start the full system:"
    echo "   1. Launch simulation: ros2 launch bringup sim_one_drone.launch.py"
    echo "   2. Launch SLAM: ros2 launch bringup slam_mapping.launch.py"
    echo "   3. Then run this script"
    echo ""
    echo "Continuing with LiDAR processor only..."
fi

echo ""
echo "🎯 Starting Enhanced LiDAR Processor..."

# Launch the enhanced LiDAR processor
python3 /home/student/git/FIREWARDENBOT/src/vision/vision/enhanced_lidar_processor.py &
LIDAR_PID=$!

echo "📡 LiDAR Processor started (PID: $LIDAR_PID)"

# Wait a moment for startup
sleep 2

echo ""
echo "📊 Available Topics:"
echo "   • /lidar/obstacles      - Obstacle detection results"
echo "   • /lidar/pointcloud     - 3D point cloud data"
echo "   • /lidar/analysis       - Scan quality analysis"
echo "   • /lidar/obstacle_points - Individual obstacle points"
echo ""

echo "🌐 Integration with UI:"
echo "   • Start ROS Bridge: python3 ui/ros-data-server.py"
echo "   • Access UI at: http://localhost:9000"
echo "   • SLAM data available at: http://localhost:8090/api/slam"
echo "   • Mapping data at: http://localhost:8090/api/mapping"
echo ""

echo "🎮 Controls:"
echo "   • Press Ctrl+C to stop"
echo "   • Monitor topics: ros2 topic list"
echo "   • View LiDAR data: ros2 topic echo /lidar/analysis"
echo ""

# Function to handle cleanup
cleanup() {
    echo ""
    echo "🛑 Shutting down Enhanced LiDAR & SLAM System..."
    
    if [ ! -z "$LIDAR_PID" ]; then
        kill $LIDAR_PID 2>/dev/null || true
        echo "   ✅ LiDAR Processor stopped"
    fi
    
    echo "🔥 Enhanced LiDAR & SLAM System shutdown complete"
    exit 0
}

# Set up signal handling
trap cleanup SIGINT SIGTERM

echo "✅ Enhanced LiDAR & SLAM System is running!"
echo "   Monitoring for LiDAR data and performing real-time analysis..."

# Keep script running and show periodic status
while true; do
    sleep 30
    
    echo ""
    echo "⏰ Status Update ($(date '+%H:%M:%S')):"
    
    # Check system health
    if check_topic "/lidar/analysis" >/dev/null 2>&1; then
        echo "   🟢 LiDAR processing active"
    else
        echo "   🟡 LiDAR processing idle"
    fi
    
    if check_topic "/map" >/dev/null 2>&1; then
        echo "   🟢 SLAM mapping active"
    else
        echo "   🟡 SLAM mapping not detected"
    fi
    
    if check_topic "/leaf_detections" >/dev/null 2>&1; then
        echo "   🟢 Leaf detection active"
    else
        echo "   🟡 Leaf detection idle"
    fi
done
