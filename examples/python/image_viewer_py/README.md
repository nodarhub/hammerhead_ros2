# Image Viewer (Python)

Real-time OpenCV viewer for stereo images, disparity maps, and depth data published by Hammerhead via ROS2.

## Installation

This example is part of the hammerhead_ros2 workspace. Build it with:

```bash
cd hammerhead_ros2
colcon build --packages-select image_viewer_py
```

## Usage

```bash
ros2 run image_viewer_py image_viewer_py <image_topic>
```

### Parameters

- `image_topic`: ROS2 topic name that provides `sensor_msgs::msg::Image` messages

### Examples

```bash
# Source the workspace
source install/setup.bash

# View raw left camera
ros2 run image_viewer_py image_viewer_py /nodar/left/image_raw

# View disparity map
ros2 run image_viewer_py image_viewer_py /nodar/disparity

# View color-blended depth
ros2 run image_viewer_py image_viewer_py /nodar/color_blended_depth/image_raw
```

## Available Image Topics

| Topic | Description |
|-------|-------------|
| `/nodar/left/image_raw` | Raw left camera |
| `/nodar/right/image_raw` | Raw right camera |
| `/nodar/left/image_rect` | Rectified left image |
| `/nodar/right/image_rect` | Rectified right image |
| `/nodar/disparity` | Disparity map |
| `/nodar/color_blended_depth/image_raw` | Color-coded depth visualization |

## Features

- Support for all image topics (raw, rectified, disparity, depth)
- Real-time display with OpenCV
- Python ROS2 implementation

## Integration with ROS2 Tools

This viewer is compatible with standard ROS2 tools:

```bash
# View with rviz2
ros2 run rviz2 rviz2

# View with rqt
ros2 run rqt_image_view rqt_image_view
```

## Troubleshooting

- **No display appears**: Check that Hammerhead is running and publishing image topics
- **Topic not found**: Verify the topic name using `ros2 topic list`
- **Build errors**: Ensure all dependencies are installed and workspace is sourced

Press `Ctrl+C` to exit the viewer. 
  