cmake_minimum_required(VERSION 3.8)
project(standard_motion_recorder)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/motion_recorder_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/motion_control.zsh DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
