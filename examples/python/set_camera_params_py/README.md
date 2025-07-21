# Camera Parameter Control

Real-time camera parameter adjustment for Hammerhead via ROS2 services.

## Overview

This example demonstrates how to control camera gain and exposure in real-time using ROS2 services. 
When ROS2 interfaces are enabled in Hammerhead's configuration, 
camera services become available for dynamic parameter adjustment during operation.

## Build

```bash
cd hammerhead_ros2
colcon build --packages-up-to set_camera_params_py
```

## Prerequisites

Ensure ROS2 interfaces are enabled in Hammerhead's `master_config.ini`.

## Usage

```bash
# Source the workspace
source install/setup.bash

# Control camera exposure
ros2 run set_camera_params_py exposure

# Control camera gain
ros2 run set_camera_params_py gain
```

## Features

- **Real-time Adjustment**: Modify camera parameters while Hammerhead is running
- **Interactive Interface**: Command-line interface for parameter control
- **Dual Control**: Separate tools for exposure and gain adjustment
- **Service-based**: Uses ROS2 services for reliable parameter setting

## Service Interface

- `/nodar/set_exposure` - Camera exposure control
- `/nodar/set_gain` - Camera gain control

Both services use `hammerhead_msgs/CameraParam.srv`

## Troubleshooting

- **Service not available**: Verify ROS2 interfaces are enabled in Hammerhead configuration
- **No response**: Check that Hammerhead is running and accessible
- **Invalid parameters**: Ensure parameter values are within valid ranges
- **Build errors**: Ensure hammerhead_msgs package is built first