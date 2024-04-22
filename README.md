# Turtlebot3 Disaster Response Guide

## Overview

This guide outlines the setup and operation of the Turtlebot3 for disaster response scenarios, utilizing ROS packages for navigation, SLAM, and object detection. Ensure both the Turtlebot and a local computer are correctly configured and connected to the same network.

## Prerequisites

- Turtlebot3 equipped with a USB camera.
- ROS environment set up on both Turtlebot3 and a local computer.
- Relevant ROS packages installed: `turtlebot3_slam`, `explore_lite`, `turtlebot3_navigation`, `usb_cam`, `apriltag_ros`.

## Setup Instructions

### Turtlebot Setup

1. **Activate the USB Camera:**
   ```bash
   roslaunch usb_cam usb_cam_node.launch
   ```

2. **Initialize Turtlebot3:**
   ```bash
   roslaunch turtlebot3_bringup turtlebot3_robot.launch
   ```

### Local Computer Setup

1. **Start ROS Core:**
   ```bash
   roscore
   ```

2. **Launch Turtlebot3 Robot:**
   ```bash
   roslaunch turtlebot3_bringup turtlebot3_robot.launch
   ```

3. **Begin Navigation:**
   ```bash
   roslaunch turtlebot3_navigation move_base.launch
   ```

4. **Initiate SLAM:**
   ```bash
   roslaunch turtlebot3_slam turtlebot3_slam.launch
   ```

5. **Start Exploration:**
   ```bash
   roslaunch explore_lite explore.launch
   ```

## ROS Packages Overview

- **`turtlebot3_slam`:** Provides launch scripts for SLAM operations using the Gmapping method.
- **`explore_lite`:** Implements a greedy frontier-based algorithm for autonomous exploration.
- **`turtlebot3_navigation`:** Contains scripts for navigation setup on the Turtlebot3.

## Detecting AprilTags in the Environment

1. **Start the USB Camera on Turtlebot3:**
   ```bash
   roslaunch usb_cam usb_cam_node.launch
   ```

2. **Launch AprilTag Detection:**
   The `apriltag_ros` package will subscribe to `usb_camera/image_raw` and `usb_cam/camera_info` to detect and decode AprilTags within the camera's view.

This setup allows the Turtlebot3 to autonomously navigate and explore a disaster site while detecting critical markers for effective response operations.
