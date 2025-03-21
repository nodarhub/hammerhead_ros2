This example is a simple OpenCV viewer for ROS2 images published by hammerhead.

To run the current example, you should ensure that this example is in a ROS2 workspace

    hammerhead_ros2_ws/
        src/
            image_viewer_py/

and then from the root of the workspace: build, install, and run:

    cd hammerhead_ros2_ws
    colcon build
    . install/setup.bash
    ros2 run image_viewer_py image_viewer_py [image_topic]

The parameter `image_topic` should be a ROS2 topic name that
provides `sensor_msgs::msg::Image` messages. For example, some image topics published by hammerhead are

    /nodar/color_blended_depth/image_raw
    /nodar/left/image_raw
    /nodar/left/image_rect
    /nodar/right/image_raw
    /nodar/right/image_rect

Hence, to view the raw images of the left camera, you could run

    ros2 run image_viewer_py image_viewer_py /nodar/left/image_raw

Note that if hammerhead is not running, then you will not see anything in the viewer.
`hammerhead` is the executable which is publishing images.
This example (`image_viewer_py`) is simply a viewer that
subscribes to those messages and displays them.

Furthermore, note that there is nothing special about this viewer or the image topics published by hammerhead.
You can also view these images in other tools like `rviz2` and `rqt`. 
  