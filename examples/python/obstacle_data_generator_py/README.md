# Obstacle Data Generator (Python)

Subscribe to obstacle detection data from Hammerhead and save it to text files for analysis using Python.

## Overview

This Python implementation demonstrates how to subscribe to `hammerhead_msgs/ObstacleData` messages from Hammerhead and save the detection data in structured text files for offline analysis and processing.

The obstacle data is represented in the XZ plane (bird's eye view), where each obstacle is defined by:
- **Bounding box**: Collection of 3D points defining the obstacle perimeter
- **Velocity vector**: Movement direction and speed (currently in development)

## Installation

This example is part of the hammerhead_ros2 workspace:

```bash
cd hammerhead_ros2
colcon build --packages-up-to obstacle_data_generator_py
```

## Usage

```bash
# Source the workspace
source install/setup.bash

# Run the obstacle data generator
ros2 run obstacle_data_generator_py obstacle_data_generator_py
```

## Features

- **Python Implementation**: Easy to modify and extend for custom processing
- **Real-time Recording**: Subscribes to live obstacle detection data
- **Structured Output**: Saves data in organized text files with headers
- **Batch Processing**: Handles multiple obstacles per detection frame
- **Timestamped Data**: Preserves timing information for analysis

## Topic Interface

### Subscribed Topics
- `/nodar/obstacle_data` - Obstacle detection messages from Hammerhead

## Output Format

The generator creates an `obstacle_data` folder containing timestamped text files:

```
obstacle_data/
    obstacle_YYYYMMDD_HHMMSS_001.txt
    obstacle_YYYYMMDD_HHMMSS_002.txt
    ...
```

Each file contains:
- **Header**: Parameter descriptions and data format
- **Data**: Obstacle bounding box points and velocity vectors

### Data Structure

Each obstacle includes:
- **Bounding box points**: 3D coordinates (X, Z plane)
- **Velocity vector**: Movement direction and speed

## Coordinate System

**Important**: Obstacle data uses XZ plane representation:
- **X axis**: Left/right relative to camera
- **Z axis**: Forward/backward from camera  
- **No Y component**: Height information not included in obstacle data

## Python Dependencies

The tool uses standard Python libraries:
- `rclpy` - ROS2 Python client library
- `hammerhead_msgs` - Custom Hammerhead message types
- Standard Python I/O libraries for file operations

## Integration

### Analysis Tools

Generated text files can be used with:
- **pandas**: For data analysis and manipulation
- **numpy**: For numerical processing
- **matplotlib**: For visualization
- **scipy**: For statistical analysis

### Example Analysis

```python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load obstacle data
data = pd.read_csv('obstacle_data/obstacle_YYYYMMDD_HHMMSS_001.txt', 
                   skiprows=5)  # Skip header

# Analyze obstacle positions
positions = data[['x', 'z']].values
velocities = data[['vel_x', 'vel_z']].values

# Create scatter plot
plt.figure(figsize=(10, 6))
plt.scatter(positions[:, 0], positions[:, 1], alpha=0.6)
plt.xlabel('X Position (m)')
plt.ylabel('Z Position (m)')
plt.title('Obstacle Positions')
plt.grid(True)
plt.show()
```

## Custom Processing

The Python implementation makes it easy to add custom processing:

```python
# Example: Add custom obstacle filtering
def filter_obstacles(obstacles):
    # Filter by distance
    filtered = []
    for obs in obstacles:
        if obs.distance < 10.0:  # Only obstacles within 10m
            filtered.append(obs)
    return filtered

# Modify the obstacle_data_generator_py.py file to include custom processing
```

## Development Notes

**Velocity Vectors**: Currently published as zeros. Future Hammerhead updates will provide non-zero velocity values for dynamic obstacle tracking.

## Troubleshooting

- **No data files**: Verify Hammerhead is running and publishing obstacle data
- **Empty files**: Check that obstacles are being detected in the camera view
- **Permission errors**: Ensure write permissions in current directory
- **Import errors**: Ensure hammerhead_msgs package is built first
- **Memory issues**: Consider processing data in smaller batches for large datasets

## File Format Details

Each output file contains:
1. **Header section**: Format description and parameter order
2. **Data section**: Comma-separated values with obstacle information
3. **Timestamp**: File creation time in filename

The Python implementation provides flexibility for custom file formats and processing pipelines.
