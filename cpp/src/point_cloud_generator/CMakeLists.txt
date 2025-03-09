cmake_minimum_required(VERSION 3.10)

project(point_cloud_generator LANGUAGES CXX)

find_package(OpenCV 4 REQUIRED COMPONENTS calib3d imgproc)
find_package(rclcpp REQUIRED)
find_package(hammerhead_msgs CONFIG REQUIRED)

add_executable(point_cloud_generator point_cloud_generator.cpp)
ament_target_dependencies(point_cloud_generator rclcpp hammerhead_msgs)
target_link_libraries(point_cloud_generator opencv_calib3d opencv_imgproc)

get_directory_property(hasParent PARENT_DIRECTORY)
set_target_properties(point_cloud_generator PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS point_cloud_generator DESTINATION lib/${PROJECT_NAME})
if (NOT hasParent)
    ament_package()
endif ()
