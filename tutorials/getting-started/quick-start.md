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

Demo datasets for Livox-mid360, VLP-16 and OS1-128 sensor [Download Link](https://drive.google.com/drive/folders/1oA0kRFIH0_8oyD32IW1vZitfxYunzdBr?usp=sharing)

For more challange dataset, feel free to download from our website [slam_mode](https://superodometry.com/iccv23_challenge_LiI) and [localization_mode](https://superodometry.com/superloc). 

You might want to convert ROS1 bag into ROS2 format using this [link](https://docs.openvins.com/dev-ros1-to-ros2.html). 

For user-defined topic names, modify `SuperOdom/super_odometry/config/$(YOUR_LiDAR_SENSOR).yaml`: 
```bash
imu_topic: "/your/imu/topic"
laser_topic: "/your/laser/topic"
```
For user-defined laser-imu extrinsics, modify `SuperOdom/super_odometry/config/$(YOUR_LiDAR_SENSOR)/$(YOUR_LiDAR_SENSOR)_calibration.yaml`: 
```bash
#Rotation from laser frame to imu frame, imu^R_laser
extrinsicRotation_imu_laser: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d  
  data: [1., 0., 0.,
        0., 1., 0.,
        0., 0., 1.]

#Translation from laser frame to imu frame, imu^T_laser
extrinsicTranslation_imu_laser: !!opencv-matrix
  rows: 3
  cols: 1
  dt: d
  data: [-0.011, -0.02329, 0.04412]
```

## Step 2: Launch SuperOdom

### Basic Launch Command

Open a new terminal and launch SuperOdom using the following command:

```bash
# Terminal 1: Launch SuperOdom
cd ~/ros2_ws
source install/setup.bash

# For Velodyne VLP-16
ros2 launch super_odometry vlp_16.launch.py

# For Ouster OS1-128
ros2 launch super_odometry os1_128.launch.py

# For Livox Mid-360
ros2 launch super_odometry livox_mid360.launch.py
```

## Step 3: Start Visualization

### Launch RViz2

Open another terminal for visualization:

```bash
# Terminal 2: Launch RViz2 with SuperOdom configuration
docker exec --privileged -it superodom-ros2 /bin/bash
source install/setup.bash
cd ~/ros2_ws/src/SuperOdom/super_odometry

# Launch RViz with pre-configured display
rviz2 -d ros2.rviz
```

## Step 4: Play Sample Data

### Run the Bag File

In a third terminal, play the sample data:

```bash
# Terminal 3: Play sample data
docker exec --privileged -it superodom-ros2 /bin/bash
source install/setup.bash
cd ~/data
ros2 bag play $(YOUR_ROS2_DATASET)
```

### Monitor the Process

While the bag is playing, monitor the system:

```bash
# Terminal 4: Monitor topics and performance
cd ~/ros2_ws
source install/setup.bash

# Check active topics
ros2 topic list

# Monitor message rates
ros2 topic hz $(Topic_Name)
```

<!-- ## Step 5: Observe the Results

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
- Error messages in terminals -->

<!-- ## Next Steps

Congratulations! You've successfully run your first SuperOdom mapping session. Here's what to explore next: -->
<!-- 
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
3. **[Contributing](../contributing/)** - Contribute to SuperOdom development -->

<!-- ## Summary

In this quick start guide, you:

✅ Launched SuperOdom with default configuration  
✅ Visualized the mapping process in RViz  
✅ Processed sample LiDAR data  
✅ Observed real-time SLAM performance  
✅ Saved mapping results  

You now have a working SuperOdom installation and understand the basic workflow. The next tutorials will help you customize SuperOdom for your specific sensors and applications. -->

---

*Next: [Sensor Configuration](../sensor-configuration/) - Learn how to configure SuperOdom for your specific LiDAR sensor.*