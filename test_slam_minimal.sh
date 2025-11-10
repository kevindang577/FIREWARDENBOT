#!/bin/bash

# Minimal SLAM test - no simulation, just SLAM toolbox
echo "Starting minimal SLAM test..."

# Source ROS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Kill any existing ROS processes
pkill -f ros2
sleep 2

# Start fresh ROS daemon
ros2 daemon stop
ros2 daemon start
sleep 2

echo "Starting SLAM toolbox only..."

# Start SLAM toolbox with minimal parameters
ros2 run slam_toolbox sync_slam_toolbox_node \
    --ros-args \
    -p use_sim_time:=false \
    -p solver_plugin:=solver_plugins::CeresSolver \
    -p ceres_linear_solver:=SPARSE_NORMAL_CHOLESKY \
    -p map_frame:=map \
    -p odom_frame:=odom \
    -p base_frame:=base_link \
    -p scan_topic:=/scan \
    -p mode:=mapping \
    -p debug_logging:=true \
    -p map_update_interval:=1.0 \
    -p resolution:=0.05 \
    -p max_laser_range:=20.0 \
    -p minimum_time_interval:=0.5 \
    -p transform_timeout:=0.2 \
    -p tf_buffer_duration:=30.0 \
    -p stack_size_to_use:=40000000 \
    -p enable_interactive_mode:=true &

SLAM_PID=$!
echo "SLAM toolbox started with PID: $SLAM_PID"

# Give SLAM time to start
sleep 5

# Check if SLAM is running
if kill -0 $SLAM_PID 2>/dev/null; then
    echo "✓ SLAM toolbox is running!"
    
    # Check ROS topics
    echo "Available topics:"
    timeout 5 ros2 topic list | grep -E "(map|scan|tf)" || echo "No topics found"
    
    # Check if map topic exists
    echo "Checking for map topic..."
    timeout 5 ros2 topic info /map || echo "No map topic"
    
    echo "SLAM test running... Press Ctrl+C to stop"
    wait $SLAM_PID
else
    echo "✗ SLAM toolbox failed to start"
fi

echo "SLAM test finished"
