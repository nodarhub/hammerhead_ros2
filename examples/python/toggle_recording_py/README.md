# Recording Toggle

Simple utility to start/stop Hammerhead recording via ROS2 service.

## Overview

This example demonstrates how to control Hammerhead's recording functionality in real-time using ROS2 services. 
When ROS2 interfaces are enabled in Hammerhead's configuration, 
a recording service becomes available for dynamic control during operation.

## Build

```bash
cd hammerhead_ros2
colcon build --packages-up-to toggle_recording_py
```

## Prerequisites

Ensure ROS2 interfaces are enabled in Hammerhead's `master_config.ini`.

## Usage

```bash
# Source the workspace
source install/setup.bash

# Run the recording toggle client
ros2 run toggle_recording_py toggle_recording_py
```

## Features

- **Real-time Control**: Start/stop recording while Hammerhead is running
- **Interactive Interface**: Command-line prompts for recording control
- **Service-based**: Uses ROS2 services for reliable recording control
- **Simple Operation**: Easy-to-use toggle functionality

## Service Interface

The service uses the `/nodar/should_record` topic with standard `std_srvs/SetBool.srv` messages.

## Use Cases

- **Remote Recording Control**: Start/stop recording from remote systems
- **Automated Data Collection**: Integrate with data collection pipelines
- **Event-triggered Recording**: Start recording based on specific conditions
- **System Integration**: Control recording from higher-level applications

## Troubleshooting

- **Service not available**: Verify ROS2 interfaces are enabled in Hammerhead configuration
- **No response**: Check that Hammerhead is running and accessible
- **Recording not starting**: Verify Hammerhead has proper permissions and disk space
- **Connection issues**: Ensure network connectivity to Hammerhead device