# Hammerhead Messages

Custom ROS2 message definitions for Hammerhead-specific data types including obstacle detection and camera parameters.

## Overview

This package defines custom message types used by Hammerhead for data that doesn't fit standard ROS2 message formats:

- **ObstacleData** - Real-time obstacle detection with bounding boxes and velocity vectors
- **CameraParam** - Camera parameter control messages
- **Custom services** - Camera parameter adjustment service definitions

## Installation

This package is part of the hammerhead_ros2 workspace:

```bash
cd hammerhead_ros2
colcon build --packages-up-to hammerhead_msgs
```

## Usage

### In C++ Projects

To use these message types in a C++ project, modify your `package.xml`:

```xml
<depend>hammerhead_msgs</depend>
```

And link to this target in your `CMakeLists.txt`:

```cmake
find_package(hammerhead_msgs REQUIRED)
ament_target_dependencies(my_binary hammerhead_msgs)
```

### In Python Projects

For Python projects, modify your `package.xml`:

```xml
<depend>hammerhead_msgs</depend>
```

And import the messages in your Python code:

```python
from hammerhead_msgs.msg import ObstacleData, Obstacle
from hammerhead_msgs.srv import CameraParam
```

## Message Types

### ObstacleData

Real-time obstacle detection information:

```cpp
#include <hammerhead_msgs/msg/obstacle_data.hpp>

// Access obstacle data
for (const auto& obstacle : msg->obstacles) {
    // Process bounding box points
    for (const auto& point : obstacle.bounding_box.points) {
        // Point has x, y, z coordinates
    }
    // Access velocity vector
    double vel_x = obstacle.velocity.x;
    double vel_z = obstacle.velocity.z;
}
```

### CameraParam Service

Camera parameter control service:

```cpp
#include <hammerhead_msgs/srv/camera_param.hpp>

// Create service client
auto client = node->create_client<hammerhead_msgs::srv::CameraParam>("camera_param");

// Create request
auto request = std::make_shared<hammerhead_msgs::srv::CameraParam::Request>();
request->param_name = "exposure";
request->param_value = 100.0;
```

## Examples

The following examples demonstrate how to use these custom message types:

- **[Camera Parameter Control](../examples/cpp/set_camera_params/README.md)** - Service client for camera parameter adjustment
- **[Obstacle Data Generator](../examples/cpp/obstacle_data_generator/README.md)** - Publisher for obstacle detection data
- **[Python Camera Params](../examples/python/set_camera_params_py/README.md)** - Python service client example

## Message Definitions

### Available Messages

| Message | Description |
|---------|-------------|
| `ObstacleData` | Container for multiple obstacle detections |
| `Obstacle` | Individual obstacle with bounding box and velocity |

### Available Services

| Service | Description |
|---------|-------------|
| `CameraParam` | Camera parameter adjustment service |

## Integration

These messages integrate seamlessly with standard ROS2 tools:

```bash
# View message definitions
ros2 interface show hammerhead_msgs/msg/ObstacleData

# Monitor obstacle data
ros2 topic echo /nodar/obstacle

# Call camera parameter service
ros2 service call /camera_param hammerhead_msgs/srv/CameraParam "{param_name: 'exposure', param_value: 100.0}"
```
