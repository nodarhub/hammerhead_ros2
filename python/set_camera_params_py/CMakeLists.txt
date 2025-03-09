cmake_minimum_required(VERSION 3.10)

project(set_camera_params_py)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()