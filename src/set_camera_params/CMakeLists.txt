cmake_minimum_required(VERSION 3.10)

project(set_camera_params LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(hammerhead_msgs CONFIG REQUIRED)

foreach (TOPIC exposure gain)
    add_executable(${TOPIC} set_camera_params.cpp)

    target_compile_definitions(${TOPIC} PUBLIC TOPIC_${TOPIC})
    ament_target_dependencies(${TOPIC} rclcpp hammerhead_msgs)
    set_target_properties(${TOPIC} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
    install(TARGETS ${TOPIC} DESTINATION lib/${PROJECT_NAME})
endforeach ()

get_directory_property(hasParent PARENT_DIRECTORY)
if (NOT hasParent)
    ament_package()
endif ()
