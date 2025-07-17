# Obstacle Data Generator

Subscribe to obstacle detection data from Hammerhead and save it to text files for analysis.

## Overview

This example demonstrates how to subscribe to `hammerhead_msgs/ObstacleData` messages from Hammerhead and save the detection data in structured text files for offline analysis and processing.

The obstacle data is represented in the XZ plane (bird's eye view), where each obstacle is defined by:
- **Bounding box**: Collection of 3D points defining the obstacle perimeter
- **Velocity vector**: Movement direction and speed (currently in development)

## Installation

This example is part of the hammerhead_ros2 workspace:

```bash
cd hammerhead_ros2
colcon build --packages-select obstacle_data_generator
```

## Usage

```bash
# Source the workspace
source install/setup.bash

# Run the obstacle data generator
ros2 run obstacle_data_generator obstacle_data_generator
```

## Features

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

## Integration

### Analysis Tools

Generated text files can be used with:
- MATLAB/Octave for numerical analysis
- Python pandas for data processing
- Excel for visualization
- Custom analysis scripts

### Example Analysis

```python
import pandas as pd
import numpy as np

# Load obstacle data
data = pd.read_csv('obstacle_data/obstacle_YYYYMMDD_HHMMSS_001.txt', 
                   skiprows=5)  # Skip header

# Analyze obstacle positions
positions = data[['x', 'z']].values
velocities = data[['vel_x', 'vel_z']].values
```

## Development Notes

**Velocity Vectors**: Currently published as zeros. Future Hammerhead updates will provide non-zero velocity values for dynamic obstacle tracking.

## Troubleshooting

- **No data files**: Verify Hammerhead is running and publishing obstacle data
- **Empty files**: Check that obstacles are being detected in the camera view
- **Permission errors**: Ensure write permissions in current directory
- **Build errors**: Ensure hammerhead_msgs package is built first

## File Format Details

Each output file contains:
1. **Header section**: Format description and parameter order
2. **Data section**: Comma-separated values with obstacle information
3. **Timestamp**: File creation time in filename
