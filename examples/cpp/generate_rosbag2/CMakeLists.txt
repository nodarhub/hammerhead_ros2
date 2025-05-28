cmake_minimum_required(VERSION 3.10)

project(generate_rosbag2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(
        ${CMAKE_SOURCE_DIR}/../../../submodules/yaml-cpp/yaml-cpp
        ${CMAKE_BINARY_DIR}/yaml-cpp-build
)


find_package(OpenCV 4 REQUIRED COMPONENTS calib3d imgcodecs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)

foreach (TARGET xyz everything)
    add_executable(${TARGET} src/${TARGET}.cpp)
    target_include_directories(${TARGET} PUBLIC include)
    target_link_libraries(
            ${TARGET}
            PUBLIC
            yaml-cpp
            opencv_calib3d
            opencv_imgcodecs
            stdc++fs)
    ament_target_dependencies(
            ${TARGET}
            PUBLIC
            rclcpp
            rosbag2_cpp
            rosbag2_storage
            sensor_msgs)
    install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})
endforeach ()

ament_package()