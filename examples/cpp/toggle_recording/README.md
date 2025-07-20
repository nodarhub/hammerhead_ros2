# Recording Toggle

Simple utility to start/stop Hammerhead recording via ROS2 service.

## Overview

This example demonstrates how to control Hammerhead's recording functionality in real-time using ROS2 services. When ROS2 interfaces are enabled in Hammerhead's configuration, a recording service becomes available for dynamic control during operation.

## Installation

This example is part of the hammerhead_ros2 workspace:

```bash
cd hammerhead_ros2
colcon build --packages-up-to toggle_recording
```

## Usage

### Prerequisites

Ensure ROS2 interfaces are enabled in Hammerhead's `master_config.ini`.

### Basic Usage

```bash
# Source the workspace
source install/setup.bash

# Run the recording toggle client
ros2 run toggle_recording toggle_recording
```

## Features

- **Real-time Control**: Start/stop recording while Hammerhead is running
- **Interactive Interface**: Command-line prompts for recording control
- **Service-based**: Uses ROS2 services for reliable recording control
- **Simple Operation**: Easy-to-use toggle functionality

## Service Interface

### Available Service
- `/nodar/should_record` - Recording control service

### Message Type
Uses standard `std_srvs/SetBool.srv`:

```cpp
#include <std_srvs/srv/set_bool.hpp>

// Example service call
auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
request->data = true;  // Start recording
// request->data = false; // Stop recording
```

## Interactive Control

```bash
ros2 run toggle_recording toggle_recording
# Follow prompts to start/stop recording
```

## Integration

### Command Line Services
You can also use ROS2 command line tools:

```bash
# Start recording
ros2 service call /nodar/should_record std_srvs/srv/SetBool "{data: true}"

# Stop recording
ros2 service call /nodar/should_record std_srvs/srv/SetBool "{data: false}"
```

### Custom Applications
Integrate recording control into your applications:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

class RecordingController : public rclcpp::Node {
public:
    RecordingController() : Node("recording_controller") {
        recording_client_ = create_client<std_srvs::srv::SetBool>("/nodar/should_record");
    }
    
    void startRecording() {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        recording_client_->async_send_request(request);
    }
    
    void stopRecording() {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        recording_client_->async_send_request(request);
    }
    
private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr recording_client_;
};
```

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