# Image Viewer

Real-time OpenCV viewer for stereo images, disparity maps, and depth data published by Hammerhead.

## Build

```bash
cd hammerhead_ros2
colcon build --packages-up-to image_viewer
```

## Usage

```bash
# Source the workspace
source install/setup.bash

ros2 run image_viewer_py image_viewer_py <image_topic>
```

### Parameters

- `image_topic`: ROS2 topic name that provides `sensor_msgs::msg::Image` messages

### Examples

```bash
# Source the workspace
source install/setup.bash

# View raw left camera
ros2 run image_viewer image_viewer /nodar/left/image_raw

# View disparity map
ros2 run image_viewer image_viewer /nodar/disparity
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
| `/nodar/topbot_raw` | Raw topbot image (left is top-half, right is bottom-half) |
| `/nodar/topbot_rect` | Rectified topbot image (left is top-half, right is bottom-half) |

## Features

- Support for all image topic types
- Real-time display with OpenCV

## Alternative Usage

You can also build and run this example using standalone CMake:

```bash
mkdir build && cd build
cmake .. && make
./image_viewer /nodar/left/image_raw
```

## Integration with ROS2 Tools

There is nothing special about the image topics published by Hammerhead.
You can also view then with tools like `rviz2` and `rqt`.

## Troubleshooting

- **No display appears**: Check that Hammerhead is running and publishing image topics
- **Topic not found**: Verify the topic name using `ros2 topic list`
- **Build errors**: Ensure all dependencies are installed and workspace is sourced

Press `Ctrl+C` to exit the viewer. 
  