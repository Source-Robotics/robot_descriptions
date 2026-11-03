# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(msg_description)
if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED)
  catkin_package()
  set(description_share ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
  find_package(ament_cmake REQUIRED)
  set(description_share share/${PROJECT_NAME})
endif()
install(DIRECTORY config launch meshes docs previews scripts
        DESTINATION ${description_share})
install(FILES msg_soft_tpu.urdf msg_solid.urdf
        README.md LICENSE sim2bot.robot.json DESTINATION ${description_share})
if(NOT "$ENV{ROS_VERSION}" STREQUAL "1")
  ament_package()
endif()
