# Point Cloud Generator

Generate 3D point clouds from compressed PointCloudSoup messages published by Hammerhead.

## Overview

This example demonstrates how to subscribe to bandwidth-efficient `PointCloudSoup` messages and convert them to standard `sensor_msgs/PointCloud2` messages for visualization and processing.

The point clouds that Hammerhead generates can overwhelm the network due to bandwidth requirements. `PointCloudSoup` messages provide a compressed representation that can be losslessly reconstructed into XYZRGB point clouds while using a fraction of the bandwidth.

## Installation

This example is part of the hammerhead_ros2 workspace:

```bash
cd hammerhead_ros2
colcon build --packages-select point_cloud_generator
```

## Usage

### Prerequisites

Configure Hammerhead to publish `PointCloudSoup` messages by modifying `master_config.ini`:

```ini
publish_point_cloud_type = 5
```

### Basic Usage

```bash
# Source the workspace
source install/setup.bash

# Run the point cloud generator
ros2 run point_cloud_generator point_cloud_generator
```

## Features

- **Bandwidth Optimization**: Converts compressed PointCloudSoup messages to standard PointCloud2
- **Downsampling**: Reduces point cloud density by factor of 10 for network efficiency
- **Real-time Processing**: Publishes reconstructed point clouds in real-time
- **Quality of Service**: Uses BEST_EFFORT QoS policy for optimal performance

## Topic Interface

### Subscribed Topics
- `/nodar/point_cloud_soup` - Compressed point cloud data from Hammerhead

### Published Topics
- `/nodar/point_cloud` - Standard ROS2 point cloud messages (`sensor_msgs/PointCloud2`)

## Visualization

View the generated point clouds in rviz2:

```bash
# Launch rviz2
ros2 run rviz2 rviz2

# Add PointCloud2 display
# Set topic to: /nodar/point_cloud
# Set Reliability Policy to: Best Effort
```

### Important rviz2 Configuration

This example publishes point clouds with `ReliabilityPolicy.BEST_EFFORT` QoS policy. In rviz2:

1. Add a PointCloud2 display
2. Set topic to `/nodar/point_cloud`
3. Change **Reliability Policy** to **Best Effort**

## Performance Tuning

### ROS2 DDS Configuration

For optimal performance, configure network interface parameters:

```bash
# Adjust IP fragmentation settings
sudo sysctl net.ipv4.ipfrag_time=3
sudo sysctl net.ipv4.ipfrag_high_thresh=536870912
```

See the [ROS2 DDS tuning guide](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html) for more details.

### Production Considerations

- **Bandwidth**: Monitor network usage when publishing PointCloud2 messages
- **Downsampling**: Current example downsamples by factor of 10 - adjust as needed
- **QoS Settings**: Consider appropriate QoS policies for your application

## Troubleshooting

- **No point clouds visible**: Verify Hammerhead is publishing PointCloudSoup messages
- **Poor performance**: Check network configuration and DDS settings
- **rviz2 display issues**: Ensure Reliability Policy is set to Best Effort
- **Build errors**: Ensure hammerhead_msgs package is built first
