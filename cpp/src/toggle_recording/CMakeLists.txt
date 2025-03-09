cmake_minimum_required(VERSION 3.10)

project(toggle_recording LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(toggle_recording toggle_recording.cpp)

ament_target_dependencies(toggle_recording rclcpp std_srvs)
set_target_properties(toggle_recording PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS toggle_recording DESTINATION lib/${PROJECT_NAME})

ament_package()
