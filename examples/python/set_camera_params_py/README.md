# Camera Parameter Control (Python)

Real-time camera parameter adjustment for Hammerhead via ROS2 services using Python.

## Overview

This Python implementation demonstrates how to control camera gain and exposure in real-time using ROS2 services. When ROS2 interfaces are enabled in Hammerhead's configuration, camera services become available for dynamic parameter adjustment during operation.

## Installation

This example is part of the hammerhead_ros2 workspace:

```bash
cd hammerhead_ros2
colcon build --packages-up-to set_camera_params_py
```

## Usage

### Prerequisites

Ensure ROS2 interfaces are enabled in Hammerhead's `master_config.ini`.

### Basic Usage

```bash
# Source the workspace
source install/setup.bash

# Control camera exposure
ros2 run set_camera_params_py exposure

# Control camera gain
ros2 run set_camera_params_py gain
```

## Features

- **Python Implementation**: Easy to modify and extend for custom applications
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

```python
from hammerhead_msgs.srv import CameraParam
import rclpy
from rclpy.node import Node

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        self.exposure_client = self.create_client(CameraParam, '/nodar/set_exposure')
        self.gain_client = self.create_client(CameraParam, '/nodar/set_gain')
    
    def set_exposure(self, value):
        request = CameraParam.Request()
        request.param_name = 'exposure'
        request.param_value = float(value)
        future = self.exposure_client.call_async(request)
        return future
```

## Interactive Control

### Exposure Control
```bash
ros2 run set_camera_params_py exposure
# Follow prompts to adjust exposure values
```

### Gain Control
```bash
ros2 run set_camera_params_py gain
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
Integrate camera control into your Python applications:

```python
import rclpy
from rclpy.node import Node
from hammerhead_msgs.srv import CameraParam

class AutoExposureNode(Node):
    def __init__(self):
        super().__init__('auto_exposure')
        self.exposure_client = self.create_client(CameraParam, '/nodar/set_exposure')
        
        # Wait for service to be available
        while not self.exposure_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Exposure service not available, waiting...')
    
    def adjust_exposure(self, brightness_level):
        # Automatic exposure adjustment based on brightness
        if brightness_level < 50:
            new_exposure = 150.0
        elif brightness_level > 200:
            new_exposure = 50.0
        else:
            new_exposure = 100.0
            
        request = CameraParam.Request()
        request.param_name = 'exposure'
        request.param_value = new_exposure
        
        future = self.exposure_client.call_async(request)
        return future
```

## Python Dependencies

The tool uses standard Python libraries:
- `rclpy` - ROS2 Python client library
- `hammerhead_msgs` - Custom Hammerhead message types
- Standard Python libraries for user input and control

## Advanced Usage

### Automated Parameter Control

```python
# Example: Exposure bracketing
def exposure_bracketing(node, base_exposure):
    exposures = [base_exposure * 0.5, base_exposure, base_exposure * 2.0]
    
    for exp in exposures:
        request = CameraParam.Request()
        request.param_name = 'exposure'
        request.param_value = exp
        
        future = node.exposure_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        # Capture image here
        time.sleep(0.1)
```

### Parameter Monitoring

```python
# Example: Monitor and log parameter changes
class ParameterMonitor(Node):
    def __init__(self):
        super().__init__('parameter_monitor')
        self.param_history = []
        
    def log_parameter_change(self, param_name, value):
        timestamp = self.get_clock().now()
        self.param_history.append({
            'timestamp': timestamp,
            'parameter': param_name,
            'value': value
        })
        self.get_logger().info(f'Parameter {param_name} set to {value}')
```

## Troubleshooting

- **Service not available**: Verify ROS2 interfaces are enabled in Hammerhead configuration
- **No response**: Check that Hammerhead is running and accessible
- **Invalid parameters**: Ensure parameter values are within valid ranges
- **Import errors**: Ensure hammerhead_msgs package is built first
- **Connection issues**: Verify network connectivity to Hammerhead device