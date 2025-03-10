# Introduction

This example demonstrates how to stop/start the recording in realtime by using the ROS2 interface for
hammerhead. Specifically, if the ROS2 interfaces are enabled in the `master_config.ini`, then
hammerhead will run a service that allows you to stop/start the recording while hammerhead is running.
This service is exposed on the topic

    nodar/should_record

and accepts the type `SetBool.srv` provided by the `std_srvs` package.

## Quick Start

Place this example in a ROS2 workspace. That is, your directory structure should look like

    hammerhead_ros2_ws/
        src/
            toggle_recording/

Then from the root of the workspace, build this example:

    cd hammerhead_ros2_ws
    colcon build

Finally, register the packages with ROS2, and then run the client, that is,

    . install/setup.bash
    ros2 run toggle_recording toggle_recording 

This will run an infinite loop which allows you to communicate with the hammerhead recording service.