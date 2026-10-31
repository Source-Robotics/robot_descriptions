cmake_minimum_required(VERSION 3.0.2)
project(par6_description)

find_package(catkin REQUIRED)
catkin_package()

install(
  DIRECTORY config launch meshes kinematics_review
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES par6_flange.urdf par6_msg.urdf par6_msg_tpu_glue.urdf par6_ssg48.urdf par6_ssg48_stock.urdf par6_pneumatic.urdf
        README.md LICENSE sim2bot.robot.json CHANGELOG.md DH.md MIGRATION.md DYNAMICS.md DYNAMICS_AUDIT.md GRASP_FRAMES.md CONTACT_PERFORMANCE.md
        KINEMATICS_COMPARISON.md kinematics_comparison.svg ACTUATION.md KINEMATIC_REVIEW.md PNEUMATIC.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
