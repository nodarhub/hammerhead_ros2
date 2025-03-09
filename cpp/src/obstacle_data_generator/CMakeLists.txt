cmake_minimum_required(VERSION 3.10)

project(obstacle_data_generator LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(hammerhead_msgs CONFIG REQUIRED)

add_executable(obstacle_data_generator obstacle_data_generator.cpp)
ament_target_dependencies(obstacle_data_generator rclcpp hammerhead_msgs)

get_directory_property(hasParent PARENT_DIRECTORY)
set_target_properties(obstacle_data_generator PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS obstacle_data_generator DESTINATION lib/${PROJECT_NAME})
if (NOT hasParent)
    ament_package()
endif ()
