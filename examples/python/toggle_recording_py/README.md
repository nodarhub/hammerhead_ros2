# Recording Toggle (Python)

Simple utility to start/stop Hammerhead recording via ROS2 service using Python.

## Overview

This Python implementation demonstrates how to control Hammerhead's recording functionality in real-time using ROS2 services. When ROS2 interfaces are enabled in Hammerhead's configuration, a recording service becomes available for dynamic control during operation.

## Installation

This example is part of the hammerhead_ros2 workspace:

```bash
cd hammerhead_ros2
colcon build --packages-select toggle_recording_py
```

## Usage

### Prerequisites

Ensure ROS2 interfaces are enabled in Hammerhead's `master_config.ini`.

### Basic Usage

```bash
# Source the workspace
source install/setup.bash

# Run the recording toggle client
ros2 run toggle_recording_py toggle_recording_py
```

## Features

- **Python Implementation**: Easy to modify and extend for custom applications
- **Real-time Control**: Start/stop recording while Hammerhead is running
- **Interactive Interface**: Command-line prompts for recording control
- **Service-based**: Uses ROS2 services for reliable recording control
- **Simple Operation**: Easy-to-use toggle functionality

## Service Interface

### Available Service
- `/nodar/should_record` - Recording control service

### Message Type
Uses standard `std_srvs/SetBool.srv`:

```python
from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node

class RecordingController(Node):
    def __init__(self):
        super().__init__('recording_controller')
        self.recording_client = self.create_client(SetBool, '/nodar/should_record')
    
    def start_recording(self):
        request = SetBool.Request()
        request.data = True
        future = self.recording_client.call_async(request)
        return future
    
    def stop_recording(self):
        request = SetBool.Request()
        request.data = False
        future = self.recording_client.call_async(request)
        return future
```

## Interactive Control

```bash
ros2 run toggle_recording_py toggle_recording_py
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
Integrate recording control into your Python applications:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class ScheduledRecording(Node):
    def __init__(self):
        super().__init__('scheduled_recording')
        self.recording_client = self.create_client(SetBool, '/nodar/should_record')
        
        # Wait for service to be available
        while not self.recording_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Recording service not available, waiting...')
    
    def record_for_duration(self, duration_seconds):
        # Start recording
        self.get_logger().info('Starting recording...')
        self.set_recording(True)
        
        # Wait for specified duration
        time.sleep(duration_seconds)
        
        # Stop recording
        self.get_logger().info('Stopping recording...')
        self.set_recording(False)
    
    def set_recording(self, enable):
        request = SetBool.Request()
        request.data = enable
        future = self.recording_client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().success:
                state = "started" if enable else "stopped"
                self.get_logger().info(f'Recording {state} successfully')
            else:
                self.get_logger().error(f'Failed to control recording: {future.result().message}')
        else:
            self.get_logger().error('Service call failed')
```

## Python Dependencies

The tool uses standard Python libraries:
- `rclpy` - ROS2 Python client library
- `std_srvs` - Standard ROS2 service message types
- Standard Python libraries for user input and control

## Advanced Usage

### Event-Triggered Recording

```python
# Example: Start recording when motion is detected
class MotionTriggeredRecording(Node):
    def __init__(self):
        super().__init__('motion_triggered_recording')
        self.recording_client = self.create_client(SetBool, '/nodar/should_record')
        
        # Subscribe to motion detection topic
        self.motion_subscription = self.create_subscription(
            Bool, '/motion_detected', self.motion_callback, 10)
        
        self.recording_active = False
    
    def motion_callback(self, msg):
        if msg.data and not self.recording_active:
            self.start_recording()
        elif not msg.data and self.recording_active:
            self.stop_recording()
```

### Automated Data Collection

```python
# Example: Record data in intervals
class IntervalRecording(Node):
    def __init__(self):
        super().__init__('interval_recording')
        self.recording_client = self.create_client(SetBool, '/nodar/should_record')
        
        # Create timer for periodic recording
        self.timer = self.create_timer(60.0, self.timer_callback)  # Every minute
        self.recording_active = False
        self.record_duration = 10.0  # Record for 10 seconds
    
    def timer_callback(self):
        if not self.recording_active:
            self.start_recording()
            # Schedule stop after record_duration
            self.create_timer(self.record_duration, self.stop_recording)
```

## Use Cases

- **Remote Recording Control**: Start/stop recording from remote Python applications
- **Automated Data Collection**: Integrate with data collection pipelines
- **Event-triggered Recording**: Start recording based on sensor inputs or conditions
- **Scheduled Recording**: Implement time-based recording schedules
- **System Integration**: Control recording from higher-level Python applications

## Troubleshooting

- **Service not available**: Verify ROS2 interfaces are enabled in Hammerhead configuration
- **No response**: Check that Hammerhead is running and accessible
- **Recording not starting**: Verify Hammerhead has proper permissions and disk space
- **Import errors**: Ensure std_srvs package is available in ROS2 environment
- **Connection issues**: Ensure network connectivity to Hammerhead device