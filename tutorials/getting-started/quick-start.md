---
layout: tutorial
title: "Quick Start Guide"
permalink: /tutorials/getting-started/quick-start/
nav_order: 3
parent: "Introduction to SuperOdom"
prev_tutorial:
  title: "Installation Guide"
  url: "/tutorials/getting-started/installation/"
---

# Quick Start Guide

This guide will help you run your first SuperOdom mapping session using sample data. By the end of this tutorial, you'll have a working SuperOdom system and understand the basic workflow.

## Prerequisites

Before starting, ensure you have:
- ✅ Completed the [Installation Guide](../installation/)
- ✅ SuperOdom workspace sourced: `source ~/superodom_ws/install/setup.bash`
- ✅ Basic familiarity with ROS2 commands

## Overview: Your First Mapping Session

We'll run SuperOdom in three steps:
1. **Download sample data** - Get a test dataset
2. **Launch SuperOdom** - Start the SLAM system
3. **Play the data** - Run the mapping process
4. **Visualize results** - See the generated map

## Step 1: Download Sample Data

### Option A: Sample ROS2 Bag File (Recommended)

```bash
# Create directory for sample data
mkdir -p ~/superodom_data
cd ~/superodom_data

# Download sample VLP-16 dataset (substitute with actual download link)
wget https://github.com/superxslam/SuperOdom/releases/download/v1.0/sample_vlp16_indoor.bag

# Alternative: Create your own test data with simulation
# We'll show how to generate test data below
```

### Option B: Generate Test Data (If no sample available)

```bash
# Install Gazebo simulation (if not already installed)
sudo apt install ros-$ROS_DISTRO-gazebo-*

# Create a simple test environment
ros2 launch super_odometry simulation_demo.launch.py
```

## Step 2: Launch SuperOdom

### Basic Launch Command

Open a new terminal and launch SuperOdom with VLP-16 configuration:

```bash
# Terminal 1: Launch SuperOdom
cd ~/superodom_ws
source install/setup.bash

# Launch with VLP-16 configuration
ros2 launch super_odometry vlp_16.launch.py
```

You should see output similar to:
```
[INFO] [featureExtraction_node]: Feature extraction node started
[INFO] [imuPreintegration_node]: IMU preintegration node started  
[INFO] [laserMapping_node]: Laser mapping node started
[INFO] [laserMapping_node]: Waiting for point cloud data...
```

### Launch File Options

SuperOdom provides several pre-configured launch files:

```bash
# For Velodyne VLP-16
ros2 launch super_odometry vlp_16.launch.py

# For Ouster OS1-128
ros2 launch super_odometry os1_128.launch.py

# For Livox Mid-360
ros2 launch super_odometry livox_mid360.launch.py

# With custom parameters
ros2 launch super_odometry vlp_16.launch.py config_file:=my_custom_config.yaml
```

## Step 3: Start Visualization

### Launch RViz

Open another terminal for visualization:

```bash
# Terminal 2: Launch RViz with SuperOdom configuration
cd ~/superodom_ws
source install/setup.bash

# Launch RViz with pre-configured display
rviz2 -d src/SuperOdom/super_odometry/ros2.rviz
```

### RViz Configuration

In RViz, you should see several display panels configured:

**Essential Displays:**
- **PointCloud2** (`/velodyne_points`) - Raw LiDAR data
- **PointCloud2** (`/laser_cloud_corner`) - Edge features  
- **PointCloud2** (`/laser_cloud_surface`) - Planar features
- **PointCloud2** (`/laser_cloud_map`) - Global map
- **Path** (`/laser_odom_path`) - Odometry trajectory
- **TF** - Coordinate frames

**Key RViz Settings:**
- **Fixed Frame**: `map`
- **Target Frame**: `base_link`
- **Point Size**: 2-3 pixels for better visibility

## Step 4: Play Sample Data

### Run the Bag File

In a third terminal, play the sample data:

```bash
# Terminal 3: Play sample data
cd ~/superodom_data
source ~/superodom_ws/install/setup.bash

# Play the bag file
ros2 bag play sample_vlp16_indoor.bag

# Optional: Play at slower rate for better observation
ros2 bag play sample_vlp16_indoor.bag --rate 0.5

# Optional: Loop the playback
ros2 bag play sample_vlp16_indoor.bag --loop
```

### Monitor the Process

While the bag is playing, monitor the system:

```bash
# Terminal 4: Monitor topics and performance
cd ~/superodom_ws
source install/setup.bash

# Check active topics
ros2 topic list

# Monitor message rates
ros2 topic hz /velodyne_points
ros2 topic hz /laser_cloud_map
ros2 topic hz /laser_odom_to_init

# Check TF tree
ros2 run tf2_tools view_frames
```

## Step 5: Observe the Results

### What You Should See

As the data plays, observe the following in RViz:

1. **Raw Point Clouds**: Streaming LiDAR data in the `/velodyne_points` topic
2. **Feature Extraction**: Colored edge (red) and surface (green) features
3. **Odometry Path**: Blue line showing the estimated trajectory
4. **Global Map**: Accumulated point cloud showing the mapped environment
5. **TF Frames**: Coordinate frame relationships

### Real-Time Performance Indicators

**Good Performance Signs:**
- Smooth trajectory path without jumps
- Consistent feature extraction (steady red/green points)
- Map accumulation without excessive drift
- Processing rate > 10 Hz

**Performance Issues:**
- Jerky or discontinuous trajectory
- Missing features in certain areas
- Excessive computational delays
- Error messages in terminals

## Step 6: Save Your Results

### Save the Generated Map

```bash
# Save point cloud map
ros2 service call /save_map std_srvs/srv/Trigger

# Save trajectory
ros2 topic echo /laser_odom_path > trajectory.txt

# Save configuration for future use
cp src/SuperOdom/super_odometry/config/vlp_16.yaml my_config.yaml
```

### Export Data for Analysis

```bash
# Convert ROS2 bag to other formats if needed
ros2 bag convert input_bag/ --output-format sqlite3

# Record your own session for later analysis
ros2 bag record -o my_mapping_session \
    /velodyne_points \
    /laser_cloud_corner \
    /laser_cloud_surface \
    /laser_cloud_map \
    /laser_odom_path
```

## Understanding the Output

### Topics Published by SuperOdom

| Topic | Type | Description |
|-------|------|-------------|
| `/laser_cloud_corner` | sensor_msgs/PointCloud2 | Edge features |
| `/laser_cloud_surface` | sensor_msgs/PointCloud2 | Planar features |
| `/laser_cloud_map` | sensor_msgs/PointCloud2 | Global map |
| `/laser_odom_path` | nav_msgs/Path | Odometry trajectory |
| `/laser_odom_to_init` | nav_msgs/Odometry | Pose estimates |
| `/tf` | tf2_msgs/TFMessage | Transform tree |

### Key Performance Metrics

Monitor these metrics for system health:

```bash
# Check processing rates
ros2 topic hz /laser_cloud_corner    # Should be ~10-20 Hz
ros2 topic hz /laser_cloud_map       # Should be ~1-5 Hz  
ros2 topic hz /laser_odom_path       # Should be ~10-20 Hz

# Monitor system resources
htop  # CPU and memory usage
```

## Quick Troubleshooting

### Common Issues and Quick Fixes

#### No Point Cloud Visualization
```bash
# Check if data is being published
ros2 topic echo /velodyne_points --max_length 100

# Verify topic remapping in launch file
ros2 param list /featureExtraction
```

#### Poor Feature Extraction
```bash
# Check feature extraction parameters
ros2 param get /featureExtraction edge_threshold
ros2 param get /featureExtraction surface_threshold

# Adjust parameters if needed (see Parameter Tuning guide)
```

#### Mapping Drift or Jumps
```bash
# Check for IMU data (if available)
ros2 topic list | grep imu

# Verify time synchronization
ros2 topic echo /clock
```

#### High CPU Usage
```bash
# Reduce point cloud density
ros2 param set /featureExtraction downsample_rate 2

# Limit processing frequency
ros2 param set /laserMapping mapping_frequency 5
```

## Next Steps

Congratulations! You've successfully run your first SuperOdom mapping session. Here's what to explore next:

### Immediate Next Steps
1. **[Sensor Configuration](../sensor-configuration/)** - Set up your actual LiDAR sensor
2. **[Parameter Tuning](../parameter-tuning/)** - Optimize performance for your environment
3. **[Real-Time Mapping](../real-time-mapping/)** - Run SuperOdom with live sensor data

### Advanced Topics
1. **[IMU Integration](../imu-integration/)** - Add IMU for improved robustness
2. **[Loop Closure](../loop-closure/)** - Enable loop detection for large-scale mapping
3. **[Multi-Session Mapping](../multi-session/)** - Continue mapping across multiple sessions

### Development and Customization
1. **[Custom Configurations](../custom-config/)** - Create configurations for new sensors
2. **[API Reference](../api-reference/)** - Understand the programmatic interface
3. **[Contributing](../contributing/)** - Contribute to SuperOdom development

## Summary

In this quick start guide, you:

✅ Launched SuperOdom with default configuration  
✅ Visualized the mapping process in RViz  
✅ Processed sample LiDAR data  
✅ Observed real-time SLAM performance  
✅ Saved mapping results  

You now have a working SuperOdom installation and understand the basic workflow. The next tutorials will help you customize SuperOdom for your specific sensors and applications.

## Sample Output

A successful mapping session should produce output similar to this:

**Console Output:**
```
[INFO] [laserMapping]: Received point cloud with 28361 points
[INFO] [laserMapping]: Extracted 847 edge features, 2156 surface features  
[INFO] [laserMapping]: Processing time: 45.2 ms
[INFO] [laserMapping]: Map contains 156,423 points
[INFO] [laserMapping]: Odometry update: x=12.34, y=5.67, z=0.12
```

**Performance Metrics:**
- Processing Rate: 15-20 Hz
- Memory Usage: ~400 MB  
- Map Points: Varies by environment size
- Accuracy: Sub-meter drift over typical indoor trajectories

---

*Next: [Sensor Configuration](../sensor-configuration/) - Learn how to configure SuperOdom for your specific LiDAR sensor.*