# Topbot Publisher

Hammerhead can consume stereo images from Lucid cameras, over ZMQ, or over ROS2. This example publishes vertically stacked stereo images (which we refer to as `topbot` images) from disk onto a ROS2 topic so that Hammerhead can consume them as a camera source.

## Hammerhead Configuration

To use this publisher with Hammerhead, set the camera source in `master_config.ini`:

```ini
camera_source = ros2:///nodar/topbots
```

The topic in the config must match the `--topic` argument (default: `/nodar/topbots`).

## Build

```bash
cd ros2/nodarhub
colcon build --packages-up-to topbot_publisher
source install/setup.bash
```

## Usage

```bash
ros2 run topbot_publisher topbot_publisher <topbot_data_directory> [--topic /nodar/topbots] [--fps 10] [--encoding auto]
```

### Parameters

- `image_dir`: Path to directory containing topbot images (default: `.`)
- `--topic`: ROS2 topic name (default: `/nodar/topbots`)
- `--fps`: Publish rate in frames per second (default: `10`)
- `--encoding`: Image encoding: `auto`, `mono8`, `mono16`, `bgr8`, `bgr16`, `bayer_bggr8`, `bayer_rggb8`, etc. (default: `auto`)
- `--loop`: Loop over the images continuously (default: off)

### Examples

```bash
# Publish topbot images with auto-detected encoding at 10 fps
ros2 run topbot_publisher topbot_publisher /path/to/topbot/data

# Publish at 5 fps on a custom topic
ros2 run topbot_publisher topbot_publisher /path/to/topbot/data --topic /camera/image_raw --fps 5

# Publish with explicit Bayer encoding, looping continuously
ros2 run topbot_publisher topbot_publisher /path/to/topbot/data --encoding bayer_rggb8 --loop
```

## Supported Image Formats

- TIFF (`.tiff`, `.tif`)
- PNG (`.png`)

## Features

- Publish pre-recorded stereo image pairs to Hammerhead over ROS2
- Automatic encoding detection from image properties
- Configurable frame rate and topic name
- Single-pass playback by default; continuous looping with `--loop`
- Per-frame timing output for performance monitoring

## Requirements

- ROS2 with `rclcpp`, `sensor_msgs`, and `cv_bridge`
- OpenCV (imgcodecs)
- Directory should contain topbot image files in a supported format

## Alternative Usage

You can also build and run this example using standalone CMake:

```bash
mkdir build && cd build
cmake .. && make
./topbot_publisher /path/to/topbot/data
```

## Troubleshooting

- **Images not found**: Ensure the directory contains TIFF or PNG files
- **Encoding errors**: Try specifying `--encoding` explicitly instead of `auto`
- **Slow publish rate**: Check per-frame timing output; large images may exceed the requested period

Press `Ctrl+C` to stop publishing.
