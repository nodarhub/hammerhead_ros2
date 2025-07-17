# Generate ROS Bag (Python)

Convert data recorded by Hammerhead into ROS2 bag format for analysis and replay using Python.

## Overview

This package provides Python tools to convert Hammerhead recorded data into standard ROS2 bag format, making it easy to analyze and replay stereo vision data with Python-based processing pipelines.

## Available Tools

### `xyz` - Point Cloud Only
Generates a bag containing only `sensor_msgs/PointCloud2` messages with points containing `x,y,z` and `r,g,b` attributes.

### `everything` - Complete Dataset
Generates a bag containing point clouds plus image data:
- Raw left and right camera images
- Rectified left camera image
- Generated point cloud data

## Installation

This example is part of the hammerhead_ros2 workspace:

```bash
cd hammerhead_ros2
colcon build --packages-select generate_rosbag2_py
```

## Usage

### Prerequisites

You need data recorded by Hammerhead in the following format:

```
20230208-133746/
    disparity/
        000000000.tiff
        000000001.tiff
        ...
    details/
        000000000.csv
        000000001.csv
        ...
```

### Basic Usage

```bash
# Source the workspace
source install/setup.bash

# Generate point cloud only bag
ros2 run generate_rosbag2_py xyz <recorded_data_directory>

# Generate complete dataset bag
ros2 run generate_rosbag2_py everything <recorded_data_directory>
```

### Examples

```bash
# Generate complete bag from recorded data
ros2 run generate_rosbag2_py everything 20230208-133746

# Save bag to custom location
ros2 run generate_rosbag2_py everything 20230208-133746 ~/Downloads/my_bag
```

## Output

The tool generates a ROS2 bag in the specified directory:

```
20230208-133746/
    bag/
        bag_0.db3  
        metadata.yaml
    disparity/
        000000000.tiff
        ...
    details/
        000000000.csv
        ...
```

## Generated Topics

### Point Cloud Topics
- `/nodar/point_cloud` - Generated 3D point cloud data

### Image Topics (with `everything` mode)
- `/nodar/left/image_raw` - Raw left camera images
- `/nodar/right/image_raw` - Raw right camera images  
- `/nodar/left/image_rect` - Rectified left camera images

## Features

- **Python Implementation**: Easy to modify and extend for custom processing
- **Disparity Processing**: Converts disparity data to 3D point clouds using OpenCV
- **Timestamp Preservation**: Maintains original timing information
- **Standard Messages**: Generates standard ROS2 message types
- **Flexible Output**: Configurable output location

## Python Dependencies

The tool uses standard Python libraries:
- `numpy` - Numerical processing
- `opencv-python` - Image processing and 3D reconstruction
- `rclpy` - ROS2 Python client library
- `sensor_msgs` - Standard ROS2 sensor message types

## Integration

Generated bags can be used with standard ROS2 tools:

```bash
# Play back the generated bag
ros2 bag play 20230208-133746/bag

# View bag info
ros2 bag info 20230208-133746/bag

# Process with Python
python3 -c "
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
# Custom processing code here
"
```

## Custom Processing

The Python implementation makes it easy to add custom processing:

```python
# Example: Add custom point cloud filtering
def process_point_cloud(points):
    # Custom filtering logic
    filtered_points = points[points[:, 2] > 0.5]  # Remove close points
    return filtered_points

# Modify the xyz.py or everything.py files to include custom processing
```

## Troubleshooting

- **Missing dependencies**: Install required Python packages: `pip install numpy opencv-python`
- **Memory issues**: Use `xyz` mode for large datasets to reduce memory usage
- **Invalid path**: Check that the recorded data directory exists and contains expected structure
- **Build errors**: Ensure all Python dependencies are installed

