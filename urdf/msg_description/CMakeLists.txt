cmake_minimum_required(VERSION 3.0.2)
project(msg_description)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch meshes docs previews DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md LICENSE sim2bot.robot.json msg_solid.urdf msg_soft_tpu.urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
