cmake_minimum_required(VERSION 3.0.2)
project(par6_description)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY config launch meshes docs previews
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES par6_flange.urdf par6_msg.urdf par6_msg_tpu_glue.urdf
        par6_ssg48.urdf par6_ssg48_stock.urdf par6_pneumatic.urdf
        README.md LICENSE sim2bot.robot.json
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
