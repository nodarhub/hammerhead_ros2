This example is a simple OpenCV viewer for ROS2 images published by hammerhead.

To build and run the example, you should be able to use either the traditional CMake process:

    mkdir build
    cd build/
    cmake ..
    make
    ./image_viewer [image_topic]

or the ROS2 process. In the latter case, you would place this directory in a ROS2 workspace's `src` folder

    hammerhead_ros2_ws/src/image_viewer/
        CMakeLists.txt
        image_viewer.cpp
        package.xml

and then build and run this example from the root of the workspace:

    cd hammerhead_ros2_ws
    colcon build
    . install/setup.bash
    ros2 run image_viewer image_viewer [image_topic]

The parameter `image_topic` should be a ROS2 topic name that
provides `sensor_msgs::msg::Image` messages. For example, some image topics published by hammerhead are

    /nodar/color_blended_depth/image_raw
    /nodar/disparity
    /nodar/left/image_raw
    /nodar/left/image_rect
    /nodar/right/image_raw
    /nodar/right/image_rect

Hence to view the raw images of the left camera, you could run either

    ./image_viewer /nodar/left/image_raw

or

    ros2 run image_viewer image_viewer /nodar/left/image_raw

depending on whether you are using the CMake or ROS2 build process.

Note that if hammerhead is not running, then you will not see anything in the viewer.
`hammerhead` is the executable which is publishing images. This example (`image_viewer`) is simply a viewer that
subscribes to those messages and displays them.

Furthermore, note that there is nothing special about this viewer or the image topics published by hammerhead. You can
also view these images in other tools like `rviz2` and `rqt`. 
  