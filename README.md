# Hammerhead ROS2

> A comprehensive ROS2 client library for interfacing with the Hammerhead stereo vision system

[![License](images/shark_license.png)](LICENSE)

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Message Types & Topics](#message-types-topics)
- [Project Structure](#project-structure)
- [Examples & Tutorials](#examples-tutorials)
- [3D Coordinate System & Point Cloud Conversion](#3d-coordinate-system-point-cloud-conversion)
- [API Reference](#api-reference)
- [Best Practices & Tips](#best-practices-tips)

## Overview

The Hammerhead system is a high-performance stereo vision processing unit
that publishes various types of data over ROS2.
This library provides easy-to-use APIs for receiving and processing:

- **Stereo Images** - Raw and rectified left/right camera feeds
- **Depth Data** - Disparity maps and color-blended depth images
- **Point Clouds** - 3D point cloud data with or without RGB information
- **Obstacle Detection** - Real-time obstacle data with bounding boxes
- **Camera Control** - Parameter adjustment and recording control

You can also use the Topbot Publisher examples to
learn how to send messages to Hammerhead for processing.

### Key Features

| Feature                    | Description                                             |
|----------------------------|---------------------------------------------------------|
| 🐍 **Python Ready**        | Complete Python packages with examples and utilities    |
| ⚡ **High Performance C++** | Optimized C++ implementation for real-time applications |
| 🔌 **ROS2 Native**         | Full ROS2 integration with standard message types       |

## Quick Start

### Repository Setup

```bash
# Get the Hammerhead ROS2 repository
git clone git@github.com:nodarhub/hammerhead_ros2.git

# If you received the HDK with version X.X.X, you can check out the corresponding tag (skip this step if you want the latest version):
git checkout X.X.X

# Make sure that the submodules are up to date
git submodule update --init --recursive
```

### ROS2 Installation & Usage

```bash
# Build the workspace
cd hammerhead_ros2
colcon build

# Source the workspace
source install/setup.bash

# Load the DDS transport config (see "DDS Transport Configuration" section below)
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/config/fastdds_shm.xml

# View live left raw image from Hammerhead
ros2 run image_viewer image_viewer /nodar/left/image_raw

# Generate point cloud data and save to rosbag
ros2 run generate_rosbag2 generate_rosbag2

# Record obstacle detection data
ros2 run obstacle_data_recorder obstacle_data_recorder
```

> Correct DDS configuration is one of the most important steps
> you can take to get Ros2 running smoothly.
> Using the Ros2 defaults, a system that is capable of 20fps
> can see performance drop under 1fps!
> See "DDS Transport Configuration" section below on how to tune this.

### Build Scripts

The convenience scripts `compile.sh` and `clean.sh` build and clean all the examples while making sure that all the
build artifacts always remain in the same place.

```bash
# Build all examples
./compile.sh

# Clean build artifacts
./clean.sh
```

## Message Types & Topics

Hammerhead publishes data using standard ROS2 message types over predefined topics:

### Image Topics

| Topic                                  | Description                     | Message Type        |
|----------------------------------------|---------------------------------|---------------------|
| `/nodar/left/image_raw`                | Raw left camera feed            | `sensor_msgs/Image` |
| `/nodar/right/image_raw`               | Raw right camera feed           | `sensor_msgs/Image` |
| `/nodar/left/image_rect`               | Rectified left image            | `sensor_msgs/Image` |
| `/nodar/right/image_rect`              | Rectified right image           | `sensor_msgs/Image` |
| `/nodar/disparity`                     | Disparity map (Q12.4 format)    | `sensor_msgs/Image` |
| `/nodar/color_blended_depth/image_raw` | Color-coded depth visualization | `sensor_msgs/Image` |
| `/nodar/topbot_raw`                    | Raw topbot image (left is top-half, right is bottom-half) | `sensor_msgs/Image` |
| `/nodar/topbot_rect`                   | Rectified topbot image (left is top-half, right is bottom-half) | `sensor_msgs/Image` |

### 3D Data Topics

| Topic                | Description             | Message Type                   |
|----------------------|-------------------------|--------------------------------|
| `/nodar/point_cloud` | 3D point cloud data     | `sensor_msgs/PointCloud2`      |
| `/nodar/obstacle`    | Obstacle detection data | `hammerhead_msgs/ObstacleData` |

### Control Topics

| Topic                 | Description              | Message Type                  |
|-----------------------|--------------------------|-------------------------------|
| `/nodar/camera_param` | Camera parameter control | `hammerhead_msgs/CameraParam` |
| `/nodar/recording`    | Recording on/off control | `std_msgs/Bool`               |

## Project Structure

The `hammerhead_msgs` folder contains custom message definitions for Hammerhead-specific data types like obstacle detection and camera parameters.

The `examples` folder contains comprehensive examples that demonstrate how to interact with Hammerhead using ROS2. We envision that you will use these examples as a jumping-off point for your application.

We suggest that you start by examining the code and README's in the individual example directories for more details about what each example does.

## Examples & Tutorials

### 🐍 Python Examples

Python examples provide easy-to-use scripts for common Hammerhead integration tasks.

#### Visualization Examples

- **[Image Viewer](examples/python/image_viewer_py/README.md)** - Real-time OpenCV viewer for stereo images, disparity maps, and depth data

#### Data Generation Examples

- **[Generate ROS Bag](examples/python/generate_rosbag2_py/README.md)** - Generate point cloud data and save to ROS2 bag files
- **[Point Cloud Generator](examples/python/point_cloud_generator_py/README.md)** - Generate 3D point clouds from stereo data
- **[Obstacle Data Recorder](examples/python/obstacle_data_recorder_py/README.md)** - Record obstacle detection data

#### Camera Source Examples

- **[Topbot Publisher](examples/python/topbot_publisher_py/README.md)** - Publish stereo topbot images from disk to Hammerhead over ROS2

#### Control Examples

- **[Camera Parameter Control](examples/python/set_camera_params_py/README.md)** - Real-time camera parameter adjustment

### ⚡ C++ Examples

High-performance C++ implementations for real-time applications and system integration.

#### Visualization Examples

- **[Image Viewer](examples/cpp/image_viewer/README.md)** - Real-time OpenCV viewer for stereo images, disparity maps, and depth data

#### Data Generation Examples

- **[Generate ROS Bag](examples/cpp/generate_rosbag2/README.md)** - Generate point cloud data and save to ROS2 bag files
- **[Point Cloud Generator](examples/cpp/point_cloud_generator/README.md)** - Generate 3D point clouds from stereo data
- **[Obstacle Data Recorder](examples/cpp/obstacle_data_recorder/README.md)** - Record obstacle detection data

#### Camera Source Examples

- **[Topbot Publisher](examples/cpp/topbot_publisher/README.md)** - Publish stereo topbot images from disk to Hammerhead over ROS2

#### Control Examples

- **[Camera Parameter Control](examples/cpp/set_camera_params/README.md)** - Real-time camera parameter adjustment

### Common Integration Workflows

#### 🎥 Image Processing Pipeline

1. Start with **Image Viewer** to verify camera feeds
2. Use **Generate ROS Bag** to capture datasets
3. Process images with custom algorithms

#### 🌐 3D Reconstruction Workflow

1. Subscribe to point cloud topics to get 3D data
2. Use **Point Cloud Generator** to create custom point clouds
3. Process with 3D algorithms
4. Integrate with navigation or mapping frameworks

#### 🚗 Obstacle Detection Integration

1. Use **Obstacle Data Recorder** to understand data format
2. Implement real-time processing of obstacle messages
3. Integrate with path planning or control systems
4. Add custom filtering or tracking algorithms

## 3D Coordinate System & Point Cloud Conversion

Hammerhead follows standard stereo reconstruction principles for converting disparity to 3D point clouds:

### Disparity Scaling

The disparity is in Q12.4 format. We scale the disparity by `1 / 16.0` to get the disparity in `float32` format:

```python
disparity_scaled = disparity.astype(np.float32) / 16.0
```

### 3D Reprojection

The scaled disparity map is reprojected into 3D space using OpenCV's `cv2.reprojectImageTo3D()` and a 4×4 reprojection
matrix `Q`:

```python
# Important: Negate the last row for correct coordinate frame
Q_corrected = Q.copy()
Q_corrected[3, :] = -Q_corrected[3, :]

# Reproject to 3D
points_3d = cv2.reprojectImageTo3D(disparity_scaled, Q_corrected)
```

A negative translation vector (`Tx < 0`) is used when creating the `Q` matrix to conform to the definition in OpenCV. This
ensures that the point cloud is generated in a consistent right-handed coordinate frame. As a result, the entire last
row of `Q` must be negated before passing to the `cv2.reprojectImageTo3D()` call.

This conversion scheme has been used in the following examples:

- **[Generate ROS Bag](examples/cpp/generate_rosbag2/README.md)** - C++ point cloud generation
- **[Point Cloud Generator](examples/cpp/point_cloud_generator/README.md)** - C++ real-time point cloud processing
- **[Generate ROS Bag](examples/python/generate_rosbag2_py/README.md)** - Python point cloud generation
- **[Point Cloud Generator](examples/python/point_cloud_generator_py/README.md)** - Python real-time point cloud processing

## API Reference

### Message Types

All Hammerhead ROS2 messages use standard ROS2 message types where possible, with custom messages defined in `hammerhead_msgs`.

#### Standard Image Messages

Used for all image data including raw stereo images, rectified images, and disparity maps.

```cpp
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/nodar/left/image_raw", 10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Process image data
        RCLCPP_INFO(this->get_logger(), "Received image: %dx%d", msg->width, msg->height);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};
```

#### ObstacleData

Contains real-time obstacle detection information with bounding boxes and velocity vectors.

```cpp
#include <hammerhead_msgs/msg/obstacle_data.hpp>
#include <rclcpp/rclcpp.hpp>

class ObstacleSubscriber : public rclcpp::Node
{
public:
    ObstacleSubscriber() : Node("obstacle_subscriber")
    {
        subscription_ = this->create_subscription<hammerhead_msgs::msg::ObstacleData>(
            "/nodar/obstacle", 10,
            std::bind(&ObstacleSubscriber::obstacle_callback, this, std::placeholders::_1));
    }

private:
    void obstacle_callback(const hammerhead_msgs::msg::ObstacleData::SharedPtr msg)
    {
        // Process obstacle data
        for (const auto& obstacle : msg->obstacles) {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected with %zu points", 
                       obstacle.bounding_box.points.size());
        }
    }
    
    rclcpp::Subscription<hammerhead_msgs::msg::ObstacleData>::SharedPtr subscription_;
};
```

## DDS Transport Configuration

When sending or receiving large images (e.g. full-resolution topbot stereo pairs),
the ROS2 middleware is almost always problematic.
We have tried several different DDS suppliers,
and our conclusion is that the default middleware is
sufficient as long as you tune it for high-throughput image transfer.
The easiest way we have found to do this is to use the XML profiles we include in the `config/` folder.

### TLDR

In the terminal where hammerhead runs AND the terminal where the sender/reciever runs,
you should run this command first:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/config/fastdds.xml
```

### Detailed Explanation

1. If sending and receiving messages on the same machine, you should to use shared memory,
   not the networking stack. The default shared memory segment seems to be
   tuned for fast, small messages. The aforementioned XML increases it by orders of magnitude.
2. If you are sending over the network, the issue is also the buffer size.
   The default size is small, meaning that large images are broken into many small packets,
   and dropping a single one of those packets then brings the whole pipeline to a halt.
   The aforementioned XML increases the buffer size so that messages are sent in fewer, larger packets.

The `config/` folder contains 2 XML profiles for tuning DDS:

- `fastdds.xml`: Tells Ros2 (technically the FASTRTPS middleware) to use shared memory by default,
  and UDP as a fallback. Both are set up with large buffer sizes.
  The builtin transports (with small buffers) are explicitly disabled.
- `fastdds_udp.xml`: Tells Ros2 to only use UDP with large buffers.

**Note:** The 8 MB buffer sizes require the kernel to allow large socket buffers. If the defaults are too small
(common on stock Linux), increase them:

```bash
# Check current limits
sysctl net.core.rmem_max net.core.wmem_max

# Set to 8 MB (must be >= the buffer sizes in the XML)
sudo sysctl -w net.core.rmem_max=8388608
sudo sysctl -w net.core.wmem_max=8388608
```

To make this persistent across reboots, add to `/etc/sysctl.conf`:

```
net.core.rmem_max=8388608
net.core.wmem_max=8388608
```

Then apply immediately with:

```bash
sudo sysctl -p
```

Without this, the kernel silently caps the buffer size and large image transfers will be slow.

### Troubleshooting

If you feel like Hammerhead is running slow, follow these steps:

1. Check that you're using FastDDS (not CycloneDDS or another middleware):

```bash
echo $RMW_IMPLEMENTATION
```

This should be empty (FastDDS is the default) or `rmw_fastrtps_cpp`. If it's set to something else
(e.g. `rmw_cyclonedds_cpp`), the XML profiles will be silently ignored. Unset it:

```bash
unset RMW_IMPLEMENTATION
```

2. Check that the XML profile is being loaded:

When a ROS2 node starts, FastDDS will print XML parsing errors to stderr if the file is malformed
or missing. If you see no errors and no improvement, the file is likely not being read. Verify:

```bash
echo $FASTRTPS_DEFAULT_PROFILES_FILE
```

If publisher and subscriber are on the same machine, then you can check
that shared memory is actually used with something like:

```bash
ls /dev/shm/ | grep fast
```

If you see `fastrtps_*` files, SHM is active. If not, FastDDS fell back to UDP - usually because
the segment size is too small for your messages.

3. If you are unsure of whether these profiles are having an effect,
   we recommend running hammerhead and the publisher subscriber 3 times...

- In the first run, try with the Ros2 defaults in the terminal where Hammerhead 
and the publisher and/or subscriber are running:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
``` 

That is the performance you would get without tuning the DDS. 

- Next, you can see what the throughput would be if Ros2 could only use UDP with large buffers 
by running this in all of your terminals:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/config/fastdds_udp.xml
``` 

- Finally, you can see what type of performance you can achieve when Ros2 is allowed to use 
shared memory for communication:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/config/fastdds.xml
``` 

We suspect that the default configs we supply will drastically increase your throughput
compared to the defaults. 

### Important Notes

- The environment variable must be set in **every terminal** that runs a ROS2 node (both publisher and subscriber).
  If only one side is configured, they will still communicate (both have UDP in common), but the unconfigured side
  becomes the bottleneck — its default buffer sizes (~512KB for SHM, ~212KB for UDP) are far too small for large
  images, and you'll see the same slow/dropped frame behavior as having no config at all.
- The publisher and subscriber should use **compatible transports**. The SHM config includes UDP as a fallback, so it
  is compatible with the UDP-only config. However, you won't get SHM benefits unless both sides have it enabled.

## Best Practices & Tips

### 🔧 Performance

- Use C++ for real-time applications
- Consider message buffering for high-frequency data
- Monitor system resources with large point clouds
- Use appropriate QoS settings for your application

### 🛡️ Reliability

- Always validate message types and versions
- Implement proper error handling
- Use ROS2 lifecycle nodes for complex applications
- Add logging for debugging

### 🌐 ROS2 Integration

- Follow ROS2 naming conventions
- Use appropriate QoS policies
- Integrate with standard ROS2 tools (rviz2, rqt)
- Consider using composition for performance

### 🔍 Debugging

- Start with simple subscribers before custom code
- Use ROS2 command-line tools for inspection
- Check topic types and message frequencies
- Use ROS2 debugging tools and visualization
