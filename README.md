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

The Hammerhead system is a high-performance stereo vision processing unit that publishes various types of data over ROS2. This library provides easy-to-use APIs for receiving and processing:

- **Stereo Images** - Raw and rectified left/right camera feeds
- **Depth Data** - Disparity maps and color-blended depth images  
- **Point Clouds** - 3D point cloud data with or without RGB information
- **Obstacle Detection** - Real-time obstacle data with bounding boxes
- **Camera Control** - Parameter adjustment and recording control

### Key Features

| Feature | Description |
|---------|-------------|
| 🐍 **Python Ready** | Complete Python packages with examples and utilities |
| ⚡ **High Performance C++** | Optimized C++ implementation for real-time applications |
| 🔌 **ROS2 Native** | Full ROS2 integration with standard message types |

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

# View live left raw image from Hammerhead
ros2 run image_viewer image_viewer /nodar/left/image_raw

# Generate point cloud data and save to rosbag
ros2 run generate_rosbag2 generate_rosbag2

# Record obstacle detection data
ros2 run obstacle_data_recorder obstacle_data_recorder
```

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
| Topic | Description | Message Type |
|-------|-------------|--------------|
| `/nodar/left/image_raw` | Raw left camera feed | `sensor_msgs/Image` |
| `/nodar/right/image_raw` | Raw right camera feed | `sensor_msgs/Image` |
| `/nodar/left/image_rect` | Rectified left image | `sensor_msgs/Image` |
| `/nodar/right/image_rect` | Rectified right image | `sensor_msgs/Image` |
| `/nodar/disparity` | Disparity map (Q12.4 format) | `sensor_msgs/Image` |
| `/nodar/color_blended_depth/image_raw` | Color-coded depth visualization | `sensor_msgs/Image` |

### 3D Data Topics
| Topic | Description | Message Type |
|-------|-------------|--------------|
| `/nodar/point_cloud` | 3D point cloud data | `sensor_msgs/PointCloud2` |
| `/nodar/obstacle` | Obstacle detection data | `hammerhead_msgs/ObstacleData` |

### Control Topics
| Topic | Description | Message Type |
|-------|-------------|--------------|
| `/nodar/camera_param` | Camera parameter control | `hammerhead_msgs/CameraParam` |
| `/nodar/recording` | Recording on/off control | `std_msgs/Bool` |

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
