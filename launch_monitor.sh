#!/bin/bash

# Source ROS 2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "🌿 Starting Leaf Detection Monitor..."
echo "This will display real-time leaf detection results"
echo "Press Ctrl+C to stop"
echo ""

# Set Python path to include our vision package
export PYTHONPATH="/home/student/git/FIREWARDENBOT/src/vision:$PYTHONPATH"

# Launch the leaf monitor
python3 /home/student/git/FIREWARDENBOT/src/vision/vision/leaf_monitor.py

