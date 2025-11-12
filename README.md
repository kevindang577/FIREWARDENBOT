# 🔥 Fire Warden Bot

An autonomous aerial firefighting system for early fire detection and prevention through automated forest inspection.

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.8+-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## 🚁 System Overview

The Fire Warden Bot combines cutting-edge computer vision, SLAM-based mapping, autonomous navigation, and real-time monitoring capabilities to create a comprehensive fire prevention solution built on the ROS 2 Humble framework.

### Key Features
- 🔍 HSV-based autumn leaf detection for fire hazard identification
- 🗺️ Real-time SLAM mapping with 0.05m resolution
- 🧭 Nav2 autonomous navigation with obstacle avoidance
- 💻 Professional web UI with real-time monitoring
- 🌍 Complete Gazebo simulation environment
- 🚀 Multi-mode launch system with error recovery

## 🏗️ System Architecture

```
[UAV Sensors: LiDAR, Camera, IMU] → /model/drone1/scan, /model/drone1/camera, /model/drone1/imu
    ↓
[SLAM Node (async_slam_toolbox_node)] → /map, /tf
    ↓                            ↘
[Computer Vision (enhanced_leaf_detector)] → /leaf_detections   → [Web UI (ros-data-server.py + HTML Interface)]
           ↑                         ↙
[Navigation (Nav2 Stack)] ← /map, /tf  ← [Mission Commands via HTTP API]
           ↓                                    ↙
      /cmd_vel → [UAV Control]           [User Interface] → /mission/goal, /mission/command, /emergency_stop
```

## 🛠️ Installation

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.8+
- Node.js (for web UI)

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-* ros-humble-gazebo-*
sudo apt install python3-opencv python3-pip nodejs npm

# Clone repository
git clone https://github.com/kevindang577/FIREWARDENBOT.git
cd FIREWARDENBOT

# Build project
colcon build
source install/setup.bash
```

## 🚀 Quick Start

### 1. Launch Full System
```bash
# Terminal 1: Launch simulation with SLAM
./launch_slam_realtime.sh

# Terminal 2: Launch computer vision
./launch_vision.sh

# Terminal 3: Launch web UI
cd ui && ./launch.sh
```

### 2. Access Web Interface
Open your browser to: `http://localhost:8080`

## 📁 Project Structure

```
├── src/
│   ├── bringup/           # Launch configurations
│   ├── vision/            # Computer vision system
│   ├── perception/        # Sensor processing
│   ├── sim/              # Simulation worlds
│   └── description/       # Robot models
├── ui/                   # Web interface
├── detection_outputs/    # Vision system outputs
├── World/               # Gazebo environments
└── docs/                # Documentation
```

## 🔧 Configuration

### SLAM Parameters
Edit `src/bringup/config/slam_params.yaml`:
- Map resolution: 0.05m
- Solver: Ceres with SPARSE_NORMAL_CHOLESKY
- Loop closure enabled

### Vision System
Configure HSV ranges in `src/vision/scripts/enhanced_leaf_detector.py`:
- Brown: [8, 50, 20] to [20, 255, 200]
- Orange: [10, 100, 100] to [25, 255, 255]
- Yellow: [25, 50, 50] to [35, 255, 255]
- Red: [0, 120, 70] to [10, 255, 255]

### Navigation
Modify Nav2 parameters in `src/bringup/config/nav2_params.yaml`

## 🎮 Usage

### Mission Types
1. **Autonomous Patrol** - Systematic area coverage
2. **Waypoint Navigation** - User-defined path following
3. **Emergency Response** - Rapid deployment
4. **Mapping Missions** - Area baseline establishment

### Web UI Controls
- **Mission Planning**: Interactive waypoint placement
- **Real-time Monitoring**: System health dashboard
- **Emergency Controls**: Stop, pause, return-home
- **Session Recording**: rosbag2 integration

## 🧪 Testing

```bash
# Run system tests
./test_vision.py
./test_vision_frames.py

# Launch individual components
ros2 launch vision vision_system.launch.py
ros2 launch bringup navigation.launch.py
ros2 launch bringup slam_mapping.launch.py
```

## 📊 Performance

- **Real-time Operation**: <100ms latency
- **Map Resolution**: 0.05m per pixel
- **Detection Rate**: 30 FPS processing
- **Web Interface**: <1s update rates

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ROS 2 Community for the robotics framework
- SLAM Toolbox developers for mapping capabilities  
- Nav2 team for navigation stack
- OpenCV community for computer vision tools

## 📞 Support

For questions and support:
- 📧 Email: [your.email@example.com]
- 🐛 Issues: [GitHub Issues](https://github.com/kevindang577/FIREWARDENBOT/issues)
- 📖 Docs: [Technical Report](COMPREHENSIVE_TECHNICAL_REPORT.md)

## 🔄 Latest Updates

- ✅ Added comprehensive system architecture documentation
- ✅ Implemented component summary table with ROS 2 topic mapping
- ✅ Updated project README with detailed installation instructions
- ✅ Enhanced .gitignore for ROS 2 project structure

---

**🔥 Fire Warden Bot - Protecting forests through autonomous technology 🌲**
