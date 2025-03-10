# Introduction

This example demonstrates how to control the camera gain and exposure in realtime by using the ROS2 interface for
hammerhead. Specifically, if the ROS2 interfaces are enabled in the `master_config.ini`, then
hammerhead will run a camera service that allows you to modify the exposure and gain while hammerhead is running.
These services are exposed on the topics

    nodar/set_exposure
    nodar/set_gain

and accept the type `CameraParam.srv` provided by the `hammerhead_msgs` package.

## Quick Start

Place this example along with the `hammerhead_msgs` folder in a ROS2 workspace. That is, your directory structure should
look like

    hammerhead_ros2_ws/
        src/
            hammerhead_msgs/
            set_camera_params_py/

Then from the root of the workspace, build the message types and this example:

    cd hammerhead_ros2_ws
    colcon build

Finally, register the packages with ROS2, and then run the exposure or gain client, that is,

    . install/setup.bash
    ros2 run set_camera_params_py exposure 

or

    . install/setup.bash
    ros2 run set_camera_params_py gain 

Both of these options will run an infinite loop which allows you to communicate with the hammerhead camera services.