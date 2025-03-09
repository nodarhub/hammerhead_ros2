cmake_minimum_required(VERSION 3.10)

project(generate_rosbag2_py)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()