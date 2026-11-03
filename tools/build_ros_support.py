# SPDX-License-Identifier: Apache-2.0
"""Generate native ROS 2 packaging/launch support and retain ROS 1 entry points."""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'tools/ros_support'
FAMILIES = ('par6', 'parol6', 'msg', 'ssg48')


def build(families=FAMILIES):
    for family in families:
        package = ROOT/f'urdf/{family}_description'
        name = package.name
        for directory in ['scripts', 'launch', 'config', 'docs']:
            (package/directory).mkdir(exist_ok=True)
        shutil.copyfile(SOURCE/'robot_description.py', package/'scripts/robot_description.py')
        shutil.copyfile(ROOT/'tools/ros_urdf.py', package/'scripts/ros_urdf.py')
        template = (SOURCE/'display.launch.py').read_text()
        (package/'launch/display.launch.py').write_text(
            template.replace('@PACKAGE@', name).replace('@FAMILY@', family), encoding='utf-8')
        tree = ET.parse(package/'package.xml'); root = tree.getroot(); root.set('format', '3')
        for entry in list(root):
            if entry.tag.endswith('_depend') or entry.tag == 'export':
                root.remove(entry)
        for version, tool in [(1, 'catkin'), (2, 'ament_cmake')]:
            ET.SubElement(root, 'buildtool_depend', condition=f'$ROS_VERSION == {version}').text = tool
        for dependency in ['joint_state_publisher', 'joint_state_publisher_gui', 'robot_state_publisher', 'python3']:
            ET.SubElement(root, 'exec_depend').text = dependency
        for version, dependencies in [(1, ['rviz']+(['gazebo_ros', 'rostopic', 'std_msgs', 'tf'] if family in ('par6','parol6') else [])),
                                      (2, ['ament_index_python', 'launch', 'launch_ros', 'rviz2'])]:
            for dependency in dependencies:
                ET.SubElement(root, 'exec_depend', condition=f'$ROS_VERSION == {version}').text = dependency
        export = ET.SubElement(root, 'export')
        for version, tool in [(1, 'catkin'), (2, 'ament_cmake')]:
            ET.SubElement(export, 'build_type', condition=f'$ROS_VERSION == {version}').text = tool
        ET.indent(tree, space='  '); tree.write(package/'package.xml', encoding='utf-8', xml_declaration=True)
        entries = ' '.join(path.name for path in sorted(package.glob(f'{family}_*.urdf')))
        (package/'CMakeLists.txt').write_text('''# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project('''+name+''')
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
install(FILES '''+entries+'''
        README.md LICENSE sim2bot.robot.json DESTINATION ${description_share})
if(NOT "$ENV{ROS_VERSION}" STREQUAL "1")
  ament_package()
endif()
''', encoding='utf-8')
        # ROS 1 also receives package:// URIs, without editing the raw URDFs.
        launch = ET.parse(package/'launch/display.launch')
        parameter = launch.find('param[@name="robot_description"]')
        argument = 'end_effector' if family in ('par6', 'parol6') else 'jaws'
        parameter.attrib.pop('textfile', None)
        parameter.set('command', f'python3 "$(find {name})/scripts/ros_urdf.py" '
                                f'"$(find {name})/{family}_$(arg {argument}).urdf" --package {name}')
        ET.indent(launch, space='  '); launch.write(package/'launch/display.launch', encoding='utf-8', xml_declaration=True)
        fixed = 'base_link' if family in ('par6', 'parol6') else 'flange'
        distance = .85 if fixed == 'base_link' else .30
        center_z = .2 if fixed == 'base_link' else .035
        center_x = .10 if fixed == 'base_link' else 0.
        (package/'config/display.rviz').write_text('''# SPDX-License-Identifier: Apache-2.0
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Value: true
      Cell Size: 0.05
      Plane Cell Count: 20
      Color: 115; 125; 140
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Value: true
      Alpha: 1
      Visual Enabled: true
      Collision Enabled: false
      Description Source: Topic
      Description Topic:
        Value: /robot_description
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
      Value: false
  Global Options:
    Background Color: 35; 50; 65
    Fixed Frame: '''+fixed+'''
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Distance: '''+str(distance)+'''
      Focal Point:
        X: '''+str(center_x)+'''
        Y: 0
        Z: '''+str(center_z)+'''
      Pitch: 0.4
      Yaw: 0.8
      Target Frame: <Fixed Frame>
Window Geometry:
  Width: 1100
  Height: 800
''', encoding='utf-8')
        shutil.copyfile(ROOT/'docs/ROS2.md', package/'docs/ROS2.md')
    print(f'Generated native ROS 2 support for {len(families)} package(s); ROS 1 entry points retained.', flush=True)


if __name__ == '__main__':
    build()
