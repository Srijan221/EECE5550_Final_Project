# Turtlebot3 Disaster Response Guide

## Overview

This guide outlines the setup and operation of the Turtlebot3 for disaster response scenarios, utilizing ROS packages for navigation, SLAM, and object detection. Ensure both the Turtlebot and a local computer are correctly configured and connected to the same network.

Video Link for the whole Pipeline in Simulation + Real TurtleBot - [Demo Video Link](https://northeastern-my.sharepoint.com/:v:/g/personal/nallaguntla_h_northeastern_edu/EQyfsn_andxGnihdCObEl7IB3qCCX-qOz7gG8-DuO9IJ_A?xsdata=MDV8MDJ8ZG9rYW5pYS5zQG5vcnRoZWFzdGVybi5lZHV8MTRiNTE4ZWQ0OTBiNGZlOGVjYjIwOGRjNjI5Y2FiNzF8YThlZWMyODFhYWEzNGRhZWFjOWI5YTM5OGI5MjE1ZTd8MHwwfDYzODQ5MzY3MjIzMzA3MDI0MnxVbmtub3dufFRXRnBiR1pzYjNkOGV5SldJam9pTUM0d0xqQXdNREFpTENKUUlqb2lWMmx1TXpJaUxDSkJUaUk2SWsxaGFXd2lMQ0pYVkNJNk1uMD18MHx8fA%3d%3d&sdata=UE5rK2taMTRreDR0OUxJRWxaY0lYYjNYbWJWc1prc0E2dnZWMnRrQXVqYz0%3d)

In the Video the turtlebot explores in both the simulation (TurtleBot House Environment) and the real world (custom made environment shown in the following figure). The AprilTags are also localized and marked on the map accurately.

*Custom Demo Environment:*
   
![Demo Environment](https://github.com/Srijan221/EECE5550_Final_Project/assets/69648635/bc8500b7-c6bb-4aff-bbe2-c802f22933d0)


## Prerequisites

- Turtlebot3 equipped with a USB camera.
- ROS environment set up on both Turtlebot3 and a local computer.
- Relevant ROS packages installed: `turtlebot3_slam`, `explore_lite`, `turtlebot3_navigation`, `usb_cam`, `apriltag_ros`.

## Setup Instructions

### Turtlebot Setup

1. **Activate the USB Camera:**
   ```bash
   rosrun usb_cam usb_cam_node
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
- **`turtlebot3_navigation`:** Contains scripts for navigation setup on the Turtlebot3 using DWA Planner

## Detecting AprilTags in the Environment

1. **Start the USB Camera on Turtlebot3:**
   ```bash
   rosrun usb_cam usb_cam_node
   ```

2. **Launch AprilTag Detection:**
   The `apriltag_ros` package will subscribe to `usb_camera/image_raw` and `usb_cam/camera_info` to detect and decode AprilTags within the camera's view.

This setup allows the Turtlebot3 to autonomously navigate and explore a disaster site while detecting critical markers for effective response operations.


