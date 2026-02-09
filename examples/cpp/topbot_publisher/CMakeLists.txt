cmake_minimum_required(VERSION 3.10)

project(topbot_publisher LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 REQUIRED COMPONENTS imgcodecs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(topbot_publisher topbot_publisher.cpp)
target_link_libraries(topbot_publisher opencv_imgcodecs stdc++fs)
ament_target_dependencies(topbot_publisher rclcpp sensor_msgs cv_bridge)

set_target_properties(topbot_publisher PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS topbot_publisher DESTINATION lib/${PROJECT_NAME})
ament_package()
