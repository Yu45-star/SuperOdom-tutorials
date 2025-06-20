---
layout: tutorial
title: "Installation Guide"
permalink: /tutorials/getting-started/installation/
nav_order: 2
parent: "Introduction to SuperOdom"
prev_tutorial:
  title: "Introduction to SuperOdom"
  url: "/tutorials/getting-started/introduction/"
next_tutorial:
  title: "Quick Start"
  url: "/tutorials/getting-started/quick-start/"
---

# Installation Guide

This guide will walk you through installing SuperOdom on your system. SuperOdom is designed for ROS2 and requires several dependencies for optimal performance.

## Building SuperOdom

### Method 1: Build from Source

### Method 2: Docker Installation (Recommended)

For users who prefer containerized deployment:

#### Prerequisites
- Docker
- NVIDIA Docker

#### Step 1: Create ROS2 Workspace and Clone Project
```bash
# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone SuperOdom
git clone https://github.com/superxslam/SuperOdom

# Clone dependencies
git clone https://github.com/Livox-SDK/livox_ros_driver2
git clone https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins
```

Make sure your directory structure looks like this:

```bash
ros2_ws/src
├── SuperOdom
├── livox_ros_driver2
└── rviz_2d_overlay_plugins
```

#### Step 2: Building a docker image
```bash
cd ~/ros2_ws/src/SuperOdom/ros2_humble_docker
docker build -t superodom-ros2:latest .
```

#### Step 3: Configure container run scripts
```bash
# Granting access to GUI
xhost +local:docker

# Edit the container run script
cd ~/ros2_ws/src/SuperOdom/ros2_humble_docker
```

Edit ``` container_run.sh ``` file and modify the following directory path:
```
PROJECT_DIR="/home/username/ros2_ws/src"  # Point to ros2_ws/src
DATASET_DIR="/home/username/dataset"  # Point to your dataset directory
```

## Next Steps

With SuperOdom successfully installed, you're ready to:

1. **[Quick Start Guide](../quick-start/)**: Run your first mapping example
2. **[Sensor Configuration](../sensor-configuration/)**: Set up your specific LiDAR sensor
3. **[Parameter Tuning](../parameter-tuning/)**: Optimize performance for your application

**Expected Performance (VLP-16 on Intel i7-8700K):**
- **Processing Rate**: 15-20 Hz
- **Memory Usage**: ~400MB
- **CPU Usage**: ~60% (single-threaded operations)

---

*Next: [Quick Start Guide](../quick-start/) - Run your first SuperOdom mapping session with sample data.*