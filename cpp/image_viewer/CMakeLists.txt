cmake_minimum_required(VERSION 3.10)

project(image_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 4 REQUIRED COMPONENTS highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(image_viewer image_viewer.cpp)
target_link_libraries(image_viewer opencv_highgui opencv_imgproc)
ament_target_dependencies(image_viewer rclcpp sensor_msgs)

set_target_properties(image_viewer PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS image_viewer DESTINATION lib/${PROJECT_NAME})
ament_package()
