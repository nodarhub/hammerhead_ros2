# Generate ROS Bag

Convert data recorded by Hammerhead into ROS2 bag format for analysis and replay.

## Overview

This package provides tools to convert Hammerhead recorded data into standard ROS2 bag format, making it easy to analyze and replay stereo vision data.

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
colcon build --packages-select generate_rosbag2
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
ros2 run generate_rosbag2 xyz <recorded_data_directory>

# Generate complete dataset bag
ros2 run generate_rosbag2 everything <recorded_data_directory>
```

### Examples

```bash
# Generate complete bag from recorded data
ros2 run generate_rosbag2 everything 20230208-133746

# Save bag to custom location
ros2 run generate_rosbag2 everything 20230208-133746 ~/Downloads/my_bag
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

- Converts disparity data to 3D point clouds using standard stereo reconstruction
- Preserves timestamps from original recording
- Generates standard ROS2 message types
- Configurable output location

## Integration

Generated bags can be used with standard ROS2 tools:

```bash
# Play back the generated bag
ros2 bag play 20230208-133746/bag

# View bag info
ros2 bag info 20230208-133746/bag

# View point clouds in rviz2
ros2 run rviz2 rviz2
```

## Troubleshooting

- **Missing disparity files**: Ensure Hammerhead recorded disparity data
- **Invalid path**: Check that the recorded data directory exists and contains the expected structure
- **Build errors**: Ensure all dependencies are installed and workspace is sourced

