# Robotic Arm with Visual Grasping

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![Python](https://img.shields.io/badge/Python-3.8+-green)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

> An intelligent robotic arm system capable of autonomous object detection, localization, and grasping using computer vision and deep learning techniques.

## 🎯 Project Overview

This project develops a robotic manipulation system that combines computer vision with robotic control to achieve autonomous grasping of household objects. The system uses an Intel RealSense depth camera for 3D scene understanding and implements real-time object detection, pose estimation, and motion planning for reliable object manipulation.

### Key Features

- **🤖 Autonomous Operation**: Complete pipeline from object detection to grasping execution
- **👁️ Computer Vision**: YOLOv8-based object detection with 6DOF pose estimation  
- **📊 Real-time Performance**: <3 second end-to-end latency with ≥10 FPS processing
- **🛡️ Safety Systems**: Emergency stop, collision avoidance, and speed limiting
- **📈 High Accuracy**: ≥80% grasp success rate with ≤2cm positioning accuracy

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   RGB-D Camera  │    │  Object Detection│    │  Pose Estimation│
│  (RealSense)    │───▶│     (YOLOv8)     │───▶│     (6DOF)      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                          │
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Arm Control    │◀───│  Motion Planning │◀───│  Grasp Planning │
│   (ROS Driver)  │    │    (MoveIt!)     │    │   (Synthesis)   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 🛠️ Hardware Components

| Component | Model | Purpose | Cost (₹) |
|-----------|-------|---------|----------|
| RGB-D Camera | Intel RealSense D435 | 3D scene reconstruction | 18,000 |
| Robotic Arm | 6-DOF Educational Arm | Object manipulation | 45,000 |
| Gripper | 2-finger Adaptive | Object grasping | 8,000 |
| Computer | NVIDIA Jetson Xavier NX | AI inference & control | 25,000 |
| **Total** | | | **₹96,000** |

## 💻 Software Stack

### Core Technologies
- **ROS Noetic** - Robot Operating System framework
- **YOLOv8** - Real-time object detection
- **MoveIt!** - Motion planning and kinematics
- **OpenCV** - Computer vision processing
- **Open3D** - 3D point cloud processing

### Dependencies
```bash
# ROS packages
ros-noetic-moveit
ros-noetic-realsense2-camera
ros-noetic-vision-msgs

# Python packages  
torch torchvision
ultralytics
opencv-python
open3d
pyrealsense2
```

## 📋 Target Objects & Performance

### Supported Objects
- **Bottle** (cylindrical, 5-15cm height)
- **Mug** (cylindrical with handle, various sizes)  
- **Box** (rectangular, rigid materials)

### Performance Targets
| Metric | Target | Current |
|--------|--------|---------|
| Grasp Success Rate | ≥80% | TBD |
| End-to-End Latency | ≤3 seconds | TBD |
| Detection FPS | ≥10 FPS | TBD |
| Position Accuracy | ≤2cm error | TBD |

## 🚀 Quick Start

### Prerequisites
- Ubuntu 20.04 with ROS Noetic
- CUDA-capable GPU (GTX 1060+ or Jetson Xavier NX)
- Intel RealSense D435 camera
- Compatible 6-DOF robotic arm

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/your-username/robotic-arm-visual-grasping.git
cd robotic-arm-visual-grasping
```

2. **Install dependencies**
```bash
# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Python requirements
pip install -r requirements.txt
```

3. **Build the workspace**
```bash
catkin_make
source devel/setup.bash
```

4. **Run calibration**
```bash
roslaunch robot_grasping calibrate_camera.launch
```

### Basic Usage

1. **Start the complete system**
```bash
roslaunch robot_grasping full_system.launch
```

2. **Run a grasping demo**
```bash
rosservice call /grasp_object "object_name: 'bottle'"
```

3. **Monitor system status**
```bash
rostopic echo /system_status
```

## 📊 Development Progress

### Phase 1: Hardware Integration ✅
- [x] Camera and arm communication setup
- [x] ROS node architecture implementation
- [x] Basic safety systems integration

### Phase 2: Perception Module 🔄
- [x] YOLOv8 model training and optimization
- [x] 6DOF pose estimation implementation
- [ ] Real-time performance optimization

### Phase 3: Planning & Control 📅
- [ ] Grasp synthesis algorithm development
- [ ] MoveIt! motion planning integration
- [ ] Collision avoidance implementation

### Phase 4: System Integration 📅
- [ ] End-to-end testing and validation
- [ ] Performance benchmarking
- [ ] Safety system validation

## 🔬 Research & Innovation

### Novel Contributions
- **Efficient Real-time Pipeline**: Optimized for low-latency operation on edge hardware
- **Robust Grasp Planning**: Adaptive grasp synthesis considering object geometry
- **Safety-first Design**: Comprehensive safety monitoring and emergency response

### Future Enhancements
- Multi-object scene manipulation
- Dynamic object tracking and grasping
- Tactile feedback integration
- Mobile platform integration

## 📈 Testing & Validation

### Test Scenarios
1. **Single Object Grasping**: Individual objects in controlled conditions
2. **Multi-object Scenes**: Object selection in cluttered environments  
3. **Lighting Variations**: Performance under different illumination
4. **Error Recovery**: System response to failed grasp attempts

### Success Metrics
- Quantitative performance against KPI targets
- Qualitative assessment of system robustness
- Safety compliance validation
- User experience evaluation

## 👥 Team

- **Hezion Wilfred** (URK23CS7006) - Team Leader, System Integration
- **Jinto Joseph** (URK24CS1210) - Computer Vision & ML
- **Basil Shaji** - Project Mentor & Guide

## 📚 Documentation

- [Requirement Analysis]([[docs/requirements.md](https://docs.google.com/document/d/1Cjd4_fvWr5iYqHqt0FfqEReUXn73dgq-Iq7RmYkoKQA/edit?usp=sharing](https://docs.google.com/document/d/e/2PACX-1vTWHIKLwAYOayBMhqomi4HyCJyX6zqyl8QpjLR8-sovp_6rvmV_aca9dDCFIUuPn8mCMpcf9qqHAqK1/pub))) - Detailed project requirements
- [System Design](docs/architecture.md) - Technical architecture overview  
- [API Reference](docs/api.md) - ROS topics, services, and messages
- [User Guide](docs/user_guide.md) - Operation and maintenance manual
- [Developer Guide](docs/dev_guide.md) - Setup and development instructions


---

**⭐ Star this repository if you find it useful!**

*Last updated: August 30, 2025*
