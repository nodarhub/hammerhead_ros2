# Obstacle Data Recorder

A recorder for obstacle data published by Hammerhead.

## Overview

This example demonstrates how to subscribe to `hammerhead_msgs/ObstacleData` messages from Hammerhead 
and save the detection data in structured text files for offline analysis and processing.
The obstacle data is represented in the XZ plane (bird's eye view), where each obstacle is defined by:

- **Bounding box**: Collection of 3D points defining the obstacle perimeter
- **Velocity vector**: Movement direction and speed (currently in development)

## Build

```bash
cd hammerhead_ros2
colcon build --packages-up-to obstacle_data_recorder_py
```

## Usage

```bash
# Source the workspace
source install/setup.bash

# Run the obstacle data recorder
ros2 run obstacle_data_recorder_py obstacle_data_recorder_py
```

## Features

- **Real-time Recording**: Subscribes to live obstacle detection data
- **Batch Processing**: Handles multiple obstacles per detection frame
- **Timestamped Data**: Preserves timing information for analysis

## Topic Interface

### Subscribed Topics
- `/nodar/obstacle_data` - Obstacle detection messages from Hammerhead

## Output Format

The recorder creates an `obstacle_data` folder containing timestamped text files:

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

## Development Notes

**Velocity Vectors**: Currently published as zeros. Future Hammerhead updates will provide non-zero velocity values for dynamic obstacle tracking.

## Troubleshooting

- **No data files**: Verify Hammerhead is running and publishing obstacle data
- **Empty files**: Check that obstacles are being detected in the camera view
- **Permission errors**: Ensure write permissions in current directory
- **Build errors**: Ensure `hammerhead_msgs` package is built first

## File Format Details

Each output file contains:

1. **Header section**: Format description and parameter order
2. **Data section**: Comma-separated values with obstacle information
3. **Timestamp**: File creation time in filename
