cmake_minimum_required(VERSION 3.10)

project(hammerhead_msgs LANGUAGES C CXX)

cmake_policy(SET CMP0077 NEW)

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(hammerhead_msg_lib INTERFACE)

set(BUILD_SHARED_LIBS ON)

rosidl_generate_interfaces(
        hammerhead_msgs
        msg/Obstacle.msg
        msg/ObstacleData.msg
        msg/PointCloudSoup.msg
        srv/CameraParam.srv
        DEPENDENCIES
        sensor_msgs
        std_msgs
)

add_dependencies(hammerhead_msg_lib hammerhead_msgs)
get_target_property(include_directories hammerhead_msgs__rosidl_typesupport_cpp INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(hammerhead_msg_lib INTERFACE ${include_directories})
target_link_libraries(hammerhead_msg_lib INTERFACE hammerhead_msgs__rosidl_typesupport_cpp)

set_target_properties(hammerhead_msg_lib PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS hammerhead_msg_lib DESTINATION lib/${PROJECT_NAME})

get_directory_property(hasParent PARENT_DIRECTORY)
if (NOT hasParent)
    ament_package()
endif ()
