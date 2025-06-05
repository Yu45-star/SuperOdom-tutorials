---
layout: tutorial
title: "Introduction to SuperOdom"
permalink: /tutorials/getting-started/introduction/
parent: Getting Started
nav_order: 1
next_tutorial:
  title: "Installation Guide"
  url: "/tutorials/getting-started/installation/"
---

# Introduction to SuperOdom

SuperOdom is a state-of-the-art LiDAR odometry and mapping system designed specifically for ROS2 environments. It provides real-time 6-DOF pose estimation and high-quality 3D mapping capabilities for various applications including autonomous vehicles, mobile robotics, and 3D mapping services.

## What Makes SuperOdom Special?

SuperOdom stands out in the crowded field of LiDAR SLAM systems through several key innovations:

### 🚀 High Performance
- **Real-time processing**: Optimized for real-time applications with up to 20Hz pose updates
- **Efficient algorithms**: Advanced feature extraction and matching algorithms
- **Multi-threading**: Parallel processing for maximum performance

### 🧩 Modular Design
- **Clean architecture**: Separate nodes for feature extraction, IMU processing, and mapping
- **Easy customization**: Modular design allows easy extension and modification
- **Plug-and-play**: Support for multiple LiDAR sensors out of the box

### 🔗 ROS2 Native
- **Modern middleware**: Built from ground up for ROS2
- **Better real-time performance**: Takes advantage of ROS2's improved real-time capabilities
- **Easy integration**: Seamless integration with ROS2 ecosystem

## System Architecture

SuperOdom consists of three main processing nodes: