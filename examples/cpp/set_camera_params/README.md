# Camera Parameter Control

Real-time camera parameter adjustment for Hammerhead via ROS2 services.

## Overview

This example demonstrates how to control camera gain and exposure in real-time using ROS2 services. When ROS2 interfaces are enabled in Hammerhead's configuration, camera services become available for dynamic parameter adjustment during operation.

## Installation

This example is part of the hammerhead_ros2 workspace:

```bash
cd hammerhead_ros2
colcon build --packages-up-to set_camera_params
```

## Usage

### Prerequisites

Ensure ROS2 interfaces are enabled in Hammerhead's `master_config.ini`.

### Basic Usage

```bash
# Source the workspace
source install/setup.bash

# Control camera exposure
ros2 run set_camera_params exposure

# Control camera gain
ros2 run set_camera_params gain
```

## Features

- **Real-time Adjustment**: Modify camera parameters while Hammerhead is running
- **Interactive Interface**: Command-line interface for parameter control
- **Dual Control**: Separate tools for exposure and gain adjustment
- **Service-based**: Uses ROS2 services for reliable parameter setting

## Service Interface

### Available Services
- `/nodar/set_exposure` - Camera exposure control
- `/nodar/set_gain` - Camera gain control

### Message Type
Both services use `hammerhead_msgs/CameraParam.srv`:

```cpp
#include <hammerhead_msgs/srv/camera_param.hpp>

// Example service call
auto request = std::make_shared<hammerhead_msgs::srv::CameraParam::Request>();
request->param_name = "exposure";
request->param_value = 100.0;
```

## Interactive Control

### Exposure Control
```bash
ros2 run set_camera_params exposure
# Follow prompts to adjust exposure values
```

### Gain Control
```bash
ros2 run set_camera_params gain
# Follow prompts to adjust gain values
```

## Integration

### Command Line Services
You can also use ROS2 command line tools:

```bash
# Set exposure via command line
ros2 service call /nodar/set_exposure hammerhead_msgs/srv/CameraParam "{param_name: 'exposure', param_value: 100.0}"

# Set gain via command line
ros2 service call /nodar/set_gain hammerhead_msgs/srv/CameraParam "{param_name: 'gain', param_value: 2.0}"
```

### Custom Applications
Integrate camera control into your applications:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <hammerhead_msgs/srv/camera_param.hpp>

class CameraController : public rclcpp::Node {
public:
    CameraController() : Node("camera_controller") {
        exposure_client_ = create_client<hammerhead_msgs::srv::CameraParam>("/nodar/set_exposure");
        gain_client_ = create_client<hammerhead_msgs::srv::CameraParam>("/nodar/set_gain");
    }
    
    void setExposure(double value) {
        auto request = std::make_shared<hammerhead_msgs::srv::CameraParam::Request>();
        request->param_name = "exposure";
        request->param_value = value;
        exposure_client_->async_send_request(request);
    }
    
private:
    rclcpp::Client<hammerhead_msgs::srv::CameraParam>::SharedPtr exposure_client_;
    rclcpp::Client<hammerhead_msgs::srv::CameraParam>::SharedPtr gain_client_;
};
```

## Troubleshooting

- **Service not available**: Verify ROS2 interfaces are enabled in Hammerhead configuration
- **No response**: Check that Hammerhead is running and accessible
- **Invalid parameters**: Ensure parameter values are within valid ranges
- **Build errors**: Ensure hammerhead_msgs package is built first