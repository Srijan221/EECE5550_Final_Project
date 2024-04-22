# Turtlebot3 Disaster Response Project


## How to run

### On Turtlebot
Open camera:
```bash
roslaunch usb_cam_node usb_cam_node.launch
```

Bringup turtlebot
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### On Local Computer
```bash
roscore
1. `roslaunch turtlebot3_bringup turtlebot3_robot.launch` on turtlebot3 to bringup the robot.
2. `roslaunch turtlebot3_navigation move_base.launch` to start navigation.
3. `roslaunch turtlebot3_slam turtlebot3_slam.launch` to start SLAM.
4. roslaunch explore_lite explore.launch` 
```


### ROS Packages

We use the following ros packages:
1. `turtlebot3_slam`: package provides roslaunch scripts for starting the SLAM. We choose `gmapping` as our SLAM method. 
2. `explore_lite`: package provides greedy frontier-based exploration. 
3. `turtlebot3_navigation`:  provides roslaunch scripts for starting the navigation. 

To control the turtlebot, we need to bringup turtlebot3. 


### Locate any apriltag in the environment

Start the USB camera on turtlebot3:
```bash
roslaunch usb_cam_node usb_cam_node.launch
```
Then, we can use `apriltag_ros` package to identify the tags in the image. `apriltag_ros` package subscribes `usb_camera/image_raw`, and `usb_cam/camera_info`.
