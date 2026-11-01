cmake_minimum_required(VERSION 3.0.2)
project(parol6_description)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY config launch meshes docs previews scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES parol6_flange.urdf parol6_msg.urdf parol6_msg_tpu_glue.urdf
        parol6_ssg48.urdf parol6_ssg48_stock.urdf parol6_pneumatic.urdf
        README.md LICENSE sim2bot.robot.json
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
