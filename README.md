# Introduction

This repo contains several examples demonstrating how to interact with Hammerhead using ROS2.

## Quick Start

You can use this repo like a ROS2 workspace. Specifically, if all the dependencies are installed, then you simply need
to clone this repo and call `colcon build` to build all the examples:

```shell
git clone git@github.com:nodarhub/hammerhead_ros2.git
cd hammerhead_ros2
colcon build
```

After that, to run a specific example, like the `image_viewer`, you would 
```shell
source install/setup.bash
ros2 run image_viewer image_viewer nodar/left/image_raw
```

Note that we provide many examples both in C++ and in Python. The python examples have a `_py` postfix. 
So, for example, the python equivalent example to the above is 
```shell
source install/setup.bash
ros2 run image_viewer_py image_viewer_py nodar/left/image_raw
```

The two convenience scripts `compile.sh` and `clean.sh` build and clean all the examples while making sure that all the
build artifacts always remain in the same place.

For more details about the examples, please refer to the individual README's. 
