This package converts the data recorded by hammerhead into a ROS2 bag. There are several examples contained herein:

1. `xyz`: Generates a bag containing only PointCloud2
   messages (https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html) with points containing
   `x,y,z` and `r,g,b` attributes.
1. `everything`: Generates a bag containing the aforemented point clouds as well as some image data:
    - The raw left and right camera images
    - The rectified left camera image

To use this example, you need to record data with hammerhead, and then
provide the output directory to this example as a command line parameter. Specifically, when recording, hammerhead will
generate a folder of data of the form:

    20230208-133746/
        disparity/
            000000000.tiff
            000000001.tiff
            ...
        details/
            000000000.csv
            000000001.csv
            ...

To run the current example, you should ensure that this example is in a ROS2 workspace

    hammerhead_ros2_ws/
        src/
            generate_rosbag2/

and then from the root of the workspace: build, install, and run one of the targets (`xyz` or `everything`) with the
path to the directory used for recording (in this case, `20230208-133746`):

    cd hammerhead_ros2_ws
    colcon build
    . install/setup.bash
    # You can run either the `xyz` or `everything` target
    # ros2 run generate_rosbag2 xyz 20230208-133746
    ros2 run generate_rosbag2 everything 20230208-133746

This will generate a `bag` folder inside the specified directory:

    20230208-133746/
                bag/
                    bag_0.db3  
                    metadata.yaml
                disparity/
                    000000000.tiff
                    000000001.tiff
                    ...
                details/
                    000000000.csv
                    000000001.csv
                    ...

Alternatively, if you want to save the bag in a different location, then you can provide that as the second argument to
the executable. For example, to save the data in a bag called `my_bag` in the `Downloads` folder:

    ros2 run generate_rosbag2 everything 20230208-133746 ~/Downloads/my_bag

