# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(par6_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch scripts docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
