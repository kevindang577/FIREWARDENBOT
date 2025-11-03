# Fire Warden Bot Computer Vision System

A comprehensive computer vision package for fire detection, object tracking, and environmental monitoring for both ground robots (Husky) and drones.

## Features

### 🔥 **Fire Detection**
- **Multi-modal Detection**: Color-based, thermal imaging, smoke detection, and ML-based methods
- **Real-time Processing**: C++ implementation for fast detection
- **Configurable Confidence**: Adjustable thresholds for different environments
- **Combination Methods**: Fuses multiple detection sources for higher accuracy

### 👁️ **Object Detection & Tracking**
- **Forest-Optimized**: Detects people, animals, vehicles in forest environments
- **Real-time Tracking**: CSRT tracker for continuous object monitoring
- **Alert System**: Configurable alerts for different object types
- **Multi-camera Support**: Processes multiple camera feeds simultaneously

### 🌐 **Stereo Vision**
- **Depth Estimation**: 3D depth maps from stereo camera pairs
- **Point Cloud Generation**: 3D environmental mapping
- **Obstacle Detection**: Real-time obstacle avoidance data

### 📸 **Image Processing**
- **Enhancement**: Contrast, brightness, and gamma correction
- **Noise Reduction**: Gaussian blur and bilateral filtering
- **Stabilization**: Optical flow-based image stabilization

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Camera Feeds  │    │  Image Processing │    │   Detection     │
│                 │───▶│                  │───▶│   Algorithms    │
│ • RGB Cameras   │    │ • Enhancement    │    │ • Fire Detection│
│ • Thermal Cams  │    │ • Noise Reduction│    │ • Object Detect │
│ • Stereo Pairs  │    │ • Stabilization  │    │ • Tracking      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                         │
┌─────────────────┐    ┌──────────────────┐             │
│      UI/Web     │◀───│  ROS Integration │◀────────────┘
│   Interface     │    │                  │
│ • Alerts        │    │ • Topics/Services│
│ • Visualizations│    │ • TF Transforms  │
│ • Control Panel │    │ • Parameter Mgmt │
└─────────────────┘    └──────────────────┘
```

## Package Structure

```
firewardenbot_vision/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── README.md                   # This file
├── config/
│   └── vision_config.yaml      # Configuration parameters
├── launch/
│   ├── vision_system.launch.py           # Main vision launch file
│   └── firewardenbot_with_vision.launch.py # Integration with existing system
├── scripts/                    # Python nodes
│   ├── fire_detection_node.py            # Main fire detection
│   ├── object_detection_node.py          # Object detection & tracking
│   ├── image_processing_node.py          # Image enhancement
│   ├── stereo_vision_node.py            # Stereo processing
│   └── vision_aggregator.py             # Result aggregation
├── src/                       # C++ nodes
│   ├── fast_fire_detector.cpp           # High-speed fire detection
│   └── object_tracker.cpp              # Real-time tracking
└── models/                    # ML models and calibration
    ├── camera_calibration/
    └── ml_models/
```

## Installation

### Prerequisites

```bash
# Install ROS 2 dependencies
sudo apt install ros-humble-cv-bridge ros-humble-image-transport ros-humble-vision-msgs

# Install OpenCV and Python packages
sudo apt install libopencv-dev python3-opencv python3-numpy python3-scipy

# Optional: Install PyTorch for ML detection
pip3 install torch torchvision

# Install ROSBridge for UI integration
sudo apt install ros-humble-rosbridge-suite
```

### Build the Package

```bash
# Navigate to your workspace
cd /home/student/git/FIREWARDENBOT

# Copy the vision package to your workspace
cp -r firewardenbot_vision World/41068_ws/src/

# Build the workspace
cd World/41068_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select firewardenbot_vision

# Source the workspace
source install/setup.bash
```

## Usage

### Quick Start

Launch the complete system with computer vision:

```bash
# For Husky robot with vision
ros2 launch firewardenbot_vision firewardenbot_with_vision.launch.py \
    robot_type:=husky world:=sparse_forest enable_vision:=true

# For Drone with vision
ros2 launch firewardenbot_vision firewardenbot_with_vision.launch.py \
    robot_type:=drone world:=open_meadows enable_vision:=true
```

### Individual Components

Launch only the vision system:

```bash
# Complete vision system
ros2 launch firewardenbot_vision vision_system.launch.py \
    robot_type:=husky \
    enable_fire_detection:=true \
    enable_object_detection:=true \
    use_cpp_detector:=true

# Fire detection only
ros2 run firewardenbot_vision fire_detection_node.py \
    --ros-args -p camera_topic:=/husky/camera/image_raw

# Fast C++ fire detector
ros2 run firewardenbot_vision fast_fire_detector \
    --ros-args -p camera_topic:=/husky/camera/image_raw
```

### Configuration

Edit `config/vision_config.yaml` to customize:

```yaml
fire_detection:
  detection_method: 'combined'  # color, thermal, smoke, ml, combined
  confidence_threshold: 0.7
  min_fire_area: 500

object_detection:
  detection_confidence: 0.5
  enable_tracking: true
  classes_of_interest: ['person', 'bear', 'car']
```

## Camera Integration

### Adding Cameras to URDF

The package includes camera sensor macros in `urdf/camera_sensors.urdf.xacro`:

```xml
<!-- RGB Camera -->
<xacro:rgb_camera name="main_camera" parent_link="base_link" 
                 xyz="0.3 0 0.1" rpy="0 0.2 0"/>

<!-- Thermal Camera -->
<xacro:thermal_camera name="thermal_camera" parent_link="base_link" 
                     xyz="0.3 0.1 0.1" rpy="0 0.2 0"/>

<!-- Stereo Cameras -->
<xacro:rgb_camera name="camera_left" parent_link="base_link" 
                 xyz="0.25 0.05 0.15" rpy="0 0 0"/>
<xacro:rgb_camera name="camera_right" parent_link="base_link" 
                 xyz="0.25 -0.05 0.15" rpy="0 0 0"/>
```

### Camera Topics

**Husky Robot:**
- Main camera: `/husky/camera/image_raw`
- Thermal camera: `/husky/thermal_camera/image_raw`
- Stereo left: `/husky/camera_left/image_raw`
- Stereo right: `/husky/camera_right/image_raw`

**Drone:**
- Main camera: `/drone/camera/image_raw`
- Thermal camera: `/drone/thermal_camera/image_raw`
- Navigation camera: `/drone/nav_camera/image_raw`
- Wide camera: `/drone/wide_camera/image_raw`

## ROS Topics

### Publications

**Fire Detection:**
- `/fire_detection/alert` (String) - Fire alert with metadata
- `/fire_detection/detected` (Bool) - Fire detection flag
- `/fire_detection/location` (PointStamped) - Fire location
- `/fire_detection/debug_image` (Image) - Annotated detection image

**Object Detection:**
- `/object_detection/detections` (Detection2DArray) - Object detections
- `/object_detection/alerts` (String) - Object alerts
- `/object_detection/debug_image` (Image) - Annotated detection image

**Aggregated Results:**
- `/firewardenbot/combined_alerts` (String) - Combined alerts from all sources
- `/firewardenbot/mission_status` (String) - Overall mission status
- `/firewardenbot/threat_level` (String) - Current threat level

### Subscriptions

**Image Inputs:**
- Camera image topics (configurable per robot)
- Thermal camera topics
- Stereo camera pairs

## Performance Optimization

### CPU vs GPU
- **C++ Detectors**: Use for real-time performance
- **Python Detectors**: More features, slightly slower
- **GPU Acceleration**: Configure OpenCV with CUDA support

### Quality vs Speed Settings

```yaml
performance:
  processing_quality: 'medium'  # low, medium, high
  max_threads: 4
  enable_gpu_acceleration: false
```

### Memory Management
- Image caching limited to prevent memory issues
- Automatic cleanup of old detection results
- Configurable processing thread pools

## Integration with UI

The vision system integrates with the Fire Warden Bot UI through ROSBridge:

1. **Start ROSBridge:**
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **Launch UI with vision:**
   ```bash
   cd /home/student/git/FIREWARDENBOT/ui
   ./launch.sh
   ```

3. **Monitor topics in browser:**
   - Fire alerts appear as toast notifications
   - Object detection updates mission status
   - Camera feeds can be displayed in UI

## Troubleshooting

### Common Issues

**1. Camera topics not found:**
```bash
# Check available topics
ros2 topic list | grep image

# Check if cameras are publishing
ros2 topic hz /husky/camera/image_raw
```

**2. OpenCV errors:**
```bash
# Install missing OpenCV packages
sudo apt install libopencv-contrib-dev

# Check OpenCV version
python3 -c "import cv2; print(cv2.__version__)"
```

**3. Performance issues:**
- Reduce image resolution in camera parameters
- Disable debug image publishing
- Use C++ detectors instead of Python
- Limit number of detection classes

**4. ROS Bridge connection:**
```bash
# Test ROSBridge connection
ros2 run rosbridge_server rosbridge_websocket

# Check WebSocket connection
curl http://localhost:9090
```

## Development

### Adding New Detection Methods

1. **Create detection class in Python:**
   ```python
   class MyCustomDetector:
       def detect(self, image):
           # Your detection logic
           return detections
   ```

2. **Add to fire_detection_node.py:**
   ```python
   if self.detection_method in ['custom', 'combined']:
       custom_detections = self.custom_detector.detect(image)
       detections.extend(custom_detections)
   ```

### Camera Calibration

1. **Collect calibration images:**
   ```bash
   ros2 run camera_calibration cameracalibrator.py \
       --size 8x6 --square 0.108 \
       image:=/husky/camera/image_raw
   ```

2. **Update camera parameters in config file**

### Machine Learning Models

To add custom ML models:

1. **Train your model** (PyTorch/TensorFlow)
2. **Save model in `models/ml_models/`**
3. **Update `MLFireDetector` class in fire_detection_node.py**
4. **Configure model path in config file**

## Contributing

1. Fork the repository
2. Create feature branch
3. Add tests for new functionality
4. Submit pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS 2 Computer Vision community
- OpenCV development team
- Fire detection research community
- Forest monitoring and wildlife conservation efforts
