#!/bin/bash

# Source ROS 2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "🍂 Starting Fire Warden Bot Computer Vision System..."
echo "This will detect and classify autumn leaves by color"
echo "Press Ctrl+C to stop"
echo ""

# Set Python path to include our vision package
export PYTHONPATH="/home/student/git/FIREWARDENBOT/src/vision:$PYTHONPATH"

# Launch the leaf detector
python3 /home/student/git/FIREWARDENBOT/src/vision/vision/leaf_detector.py

