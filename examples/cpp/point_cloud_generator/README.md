# Point Cloud Generator

Generate ROS2 `PointCloud2` messages from the `PointCloudSoup` messages published by Hammerhead.

## Overview

This example demonstrates how to subscribe to bandwidth-efficient `PointCloudSoup` messages
and convert them to standard `sensor_msgs/PointCloud2` messages for visualization and processing.
The point clouds that Hammerhead generates can overwhelm the network due to bandwidth requirements. 
`PointCloudSoup` messages provide a compressed representation that can be losslessly reconstructed 
into XYZRGB point clouds while using a fraction of the bandwidth.

## Build

```bash
cd hammerhead_ros2
colcon build --packages-up-to point_cloud_generator
```

## Prerequisites

Configure Hammerhead to publish `PointCloudSoup` messages by modifying `master_config.ini`:

```ini
publish_point_cloud_type = 5
```

## Usage

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

### Important rviz2 Configuration

This example publishes point clouds with `ReliabilityPolicy.BEST_EFFORT` QoS policy. In rviz2:

1. Add a PointCloud2 display
2. Set topic to `/nodar/point_cloud`
3. Change **Reliability Policy** to **Best Effort**

## Performance Tuning

### ROS2 DDS Configuration

If you are having networking issues, 
please refer to the [ROS2 DDS tuning guide](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html). 
For example, you may want to modify the fragmentation settings: 

```bash
# Adjust IP fragmentation settings
sudo sysctl net.ipv4.ipfrag_time=3
sudo sysctl net.ipv4.ipfrag_high_thresh=536870912
```


### Production Considerations

- **Bandwidth**: Monitor network usage when publishing PointCloud2 messages
- **Downsampling**: Current example downsamples by factor of 10 - adjust as needed
- **QoS Settings**: Consider appropriate QoS policies for your application

## Troubleshooting

- **No point clouds visible**: Verify Hammerhead is publishing PointCloudSoup messages
- **Poor performance**: Check network configuration and DDS settings
- **rviz2 display issues**: Ensure Reliability Policy is set to Best Effort
- **Build errors**: Ensure hammerhead_msgs package is built first
