This project (`hammerhead_msgs`) defines the custom message types used by hammerhead.

To use these message types in a new project, you should modify your `package.xml` to include the line:

    <depend>hammerhead_msgs</depend>

and link to this target in your `CMakeLists.txt`:

    find_package(hammerhead_msgs CONFIG REQUIRED)
    ament_target_dependencies(my_binary hammerhead_msgs)

Some other examples that we provide, such as the `set_camera_params` package, demonstrate home to do this.
