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

## Prerequisites

### System Requirements

**Operating System**: Ubuntu 20.04 LTS (Focal) or Ubuntu 22.04 LTS (Jammy)  
**Architecture**: x86_64 (AMD64)  
**Memory**: Minimum 8GB RAM (16GB recommended for large-scale mapping)  
**Storage**: At least 5GB free space for installation and dependencies  

### ROS2 Installation

SuperOdom supports the following ROS2 distributions:
- **ROS2 Humble** (Ubuntu 22.04) - Recommended
- **ROS2 Iron** (Ubuntu 22.04)
- **ROS2 Rolling** (Latest)

#### Install ROS2 Humble (Recommended)

```bash
# Set up ROS2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop-full

# Set up environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional tools
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

### Core Dependencies

SuperOdom requires several C++ libraries for mathematical operations, optimization, and point cloud processing:

```bash
# Essential build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config

# Mathematical libraries
sudo apt install -y \
    libeigen3-dev \
    libceres-dev \
    libgflags-dev \
    libgoogle-glog-dev

# Point cloud processing
sudo apt install -y \
    libpcl-dev \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-ros

# Threading and optimization
sudo apt install -y \
    libtbb-dev \
    libopencv-dev

# ROS2 additional packages
sudo apt install -y \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-image-transport
```

### LiDAR Driver Dependencies

Depending on your LiDAR sensor, install the appropriate drivers:

#### Velodyne LiDAR
```bash
sudo apt install -y \
    ros-$ROS_DISTRO-velodyne \
    ros-$ROS_DISTRO-velodyne-pointcloud
```

#### Ouster LiDAR
```bash
# Install from source (more recent than apt packages)
cd ~/ros2_ws/src
git clone https://github.com/ouster-lidar/ouster-ros.git
cd ~/ros2_ws
colcon build --packages-select ouster_msgs ouster_ros
```

#### Livox LiDAR
```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2
```

## Building SuperOdom

### Method 1: Build from Source (Recommended)

#### Step 1: Create Workspace
```bash
# Create ROS2 workspace
mkdir -p ~/superodom_ws/src
cd ~/superodom_ws/src
```

#### Step 2: Clone Repository
```bash
# Clone SuperOdom repository
git clone https://github.com/superxslam/SuperOdom.git
cd SuperOdom
git checkout ros2  # Make sure you're on the ROS2 branch

# Verify the directory structure
ls -la super_odometry/
```

#### Step 3: Install Dependencies
```bash
cd ~/superodom_ws

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Check for missing dependencies
rosdep check --from-paths src --ignore-src
```

#### Step 4: Build the Package
```bash
# Build SuperOdom
colcon build --packages-select super_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Add to bashrc for persistence
echo "source ~/superodom_ws/install/setup.bash" >> ~/.bashrc
```

### Method 2: Docker Installation (Alternative)

For users who prefer containerized deployment:

#### Create Dockerfile
```dockerfile
FROM ros:humble-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libceres-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libpcl-dev \
    libtbb-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /opt/superodom_ws
COPY . src/SuperOdom/

# Build SuperOdom
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select super_odometry

# Setup entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /opt/superodom_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
```

#### Build and Run Docker Container
```bash
# Build Docker image
docker build -t superodom:latest .

# Run container
docker run -it --net=host --privileged \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    superodom:latest
```

## Verification

### Test Installation
```bash
# Verify package installation
ros2 pkg list | grep super_odometry

# Expected output: super_odometry

# Check executables
ros2 pkg executables super_odometry

# Expected output:
# super_odometry featureExtraction_node
# super_odometry imuPreintegration_node
# super_odometry laserMapping_node
```

### Run Basic Tests
```bash
# Test feature extraction node
ros2 run super_odometry featureExtraction_node --ros-args --help

# Test IMU preintegration node
ros2 run super_odometry imuPreintegration_node --ros-args --help

# Test laser mapping node
ros2 run super_odometry laserMapping_node --ros-args --help
```

### Launch File Verification
```bash
# Check available launch files
ros2 pkg prefix super_odometry
find $(ros2 pkg prefix super_odometry) -name "*.launch.py"

# Test launch file
ros2 launch super_odometry vlp_16.launch.py --show-args
```

## Troubleshooting

### Common Issues and Solutions

#### Issue 1: Build Errors Related to Eigen
```bash
# Solution: Install development version
sudo apt install libeigen3-dev

# Or specify Eigen path explicitly
colcon build --packages-select super_odometry --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DEigen3_DIR=/usr/lib/cmake/eigen3
```

#### Issue 2: PCL Not Found
```bash
# Solution: Install PCL development packages
sudo apt install libpcl-dev ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions

# Verify PCL installation
pkg-config --modversion pcl_common-1.12
```

#### Issue 3: Ceres Solver Issues
```bash
# Solution: Install Ceres from source for latest version
cd /tmp
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

#### Issue 4: TBB Library Not Found
```bash
# Solution: Install Intel TBB
sudo apt install libtbb-dev

# For Ubuntu 22.04, you might need:
sudo apt install libtbb2-dev
```

### Build Optimization Tips

#### Faster Compilation
```bash
# Use multiple cores for compilation
colcon build --packages-select super_odometry --parallel-workers $(nproc)

# For debug builds (development)
colcon build --packages-select super_odometry --cmake-args -DCMAKE_BUILD_TYPE=Debug

# For optimized release builds (production)
colcon build --packages-select super_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Memory-Efficient Building
```bash
# Limit parallel jobs if you have limited RAM
colcon build --packages-select super_odometry --parallel-workers 2
```

## Environment Setup

### Permanent Environment Configuration
Add these lines to your `~/.bashrc` for automatic setup:

```bash
# ROS2 Environment
source /opt/ros/humble/setup.bash

# SuperOdom Workspace
source ~/superodom_ws/install/setup.bash

# Optional: Set default RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Optional: Optimize for real-time performance
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_COLORIZED_OUTPUT=1
```

### Runtime Configuration
```bash
# Apply changes
source ~/.bashrc

# Verify environment
echo $ROS_DISTRO
echo $AMENT_PREFIX_PATH | grep superodom
```

## Next Steps

With SuperOdom successfully installed, you're ready to:

1. **[Quick Start Guide](../quick-start/)**: Run your first mapping example
2. **[Sensor Configuration](../sensor-configuration/)**: Set up your specific LiDAR sensor
3. **[Parameter Tuning](../parameter-tuning/)**: Optimize performance for your application

## Performance Verification

### System Performance Check
```bash
# Check available CPU cores
nproc

# Check available memory
free -h

# Check GPU availability (optional, for visualization)
nvidia-smi  # If using NVIDIA GPU
```

### SuperOdom Performance Test
```bash
# Run performance benchmark (with sample data)
ros2 launch super_odometry benchmark.launch.py

# Monitor resource usage
htop  # In separate terminal during operation
```

**Expected Performance (VLP-16 on Intel i7-8700K):**
- **Processing Rate**: 15-20 Hz
- **Memory Usage**: ~400MB
- **CPU Usage**: ~60% (single-threaded operations)

---

*Next: [Quick Start Guide](../quick-start/) - Run your first SuperOdom mapping session with sample data.*