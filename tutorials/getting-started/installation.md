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
- [Docker](https://www.docker.com/)
- [NVIDIA Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

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

#### Step 2: Building a Docker Image
```bash
cd ~/ros2_ws/src/SuperOdom/ros2_humble_docker
docker build -t superodom-ros2:latest .
```

#### Step 3: Configure Container Run Scripts
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

#### Step 4: Launching a Docker Container
```bash
# Giving script execution privileges
sudo chmod +x container_run.sh
```

```bash
# Add current user to docker group 
sudo usermod -aG docker $USER

# Re-login or reboot to make the changes take effect
# Or run the following command to refresh group membership 
newgrp docker

# Verify success 
docker ps
```

```bash
# Launch a docker container
./container_run.sh superodom-ros2 superodom-ros2:latest
```

#### Step 5: Build Workspace in the Container
After entering the container:

```bash
# Activate the ROS2 environment 
source /opt/ros/humble/setup.bash

# First build livox_ros_driver2 
cd ~/ros2_ws/src/livox_ros_driver2 
. /build.sh humble

# Build the entire workspace 
cd ~/ros2_ws 
colcon build

# Activate the workspace 
source install/setup.bash
```

#### Step 6: Verify Installation 
```bash
# Check that the SuperOdom package is properly installed 
ros2 pkg list | grep super

# Check that your dataset is properly mounted 
ls ~/data
```

<!-- ## Next Steps

With SuperOdom successfully installed, you're ready to:

1. **[Quick Start Guide](../quick-start/)**: Run your first mapping example
2. **[Sensor Configuration](../sensor-configuration/)**: Set up your specific LiDAR sensor
3. **[Parameter Tuning](../parameter-tuning/)**: Optimize performance for your application -->

---

*Next: [Quick Start Guide](../quick-start/) - Run your first SuperOdom mapping session with sample data.*