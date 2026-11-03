# SPDX-License-Identifier: Apache-2.0
"""Display one installed robot/gripper with optional joint sliders."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'parol6_description'
FAMILY = 'parol6'


def launch_nodes(context):
    share = Path(get_package_share_directory(PACKAGE))
    spec = importlib.util.spec_from_file_location('source_robot_description', share/'scripts/robot_description.py')
    helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    value = lambda name: LaunchConfiguration(name).perform(context)
    end_effector = value('end_effector') if FAMILY in ('par6', 'parol6') else 'msg'
    entry = helper.select_model(FAMILY, end_effector, value('jaws'))
    description = ParameterValue(helper.load_description(share/entry, PACKAGE,
                                 rviz_materials=value('rviz_materials') == 'true'), value_type=str)
    parameters = {'robot_description': description, 'use_sim_time': value('use_sim_time') == 'true'}
    nodes = [LogInfo(msg=f'{FAMILY.upper()}: displaying {entry}'),
             Node(package='robot_state_publisher', executable='robot_state_publisher',
                  name='robot_state_publisher', parameters=[parameters], output='screen')]
    if value('publish_joint_states') == 'true':
        publisher = 'joint_state_publisher_gui' if value('gui') == 'true' else 'joint_state_publisher'
        nodes.append(Node(package=publisher, executable=publisher, name=publisher,
                          parameters=[dict(parameters, use_mimic_tags=True)], output='screen'))
    if value('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                          arguments=['-d', value('rviz_config')],
                          parameters=[{'use_sim_time': parameters['use_sim_time']}], output='screen'))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory(PACKAGE))
    arm = FAMILY in ('par6', 'parol6')
    options = [DeclareLaunchArgument('jaws', default_value='default' if arm else 'solid',
                 choices=['default', 'solid', 'soft_tpu', 'fin_ray'] if arm else
                         ['solid', 'soft_tpu' if FAMILY == 'msg' else 'fin_ray'])]
    if arm:
        options.insert(0, DeclareLaunchArgument('end_effector', default_value='msg',
            choices=['flange', 'msg', 'ssg48', 'pneumatic', 'msg_tpu_glue', 'ssg48_stock']))
    for name, default, help_text in [
        ('gui', 'true', 'Use joint sliders; false publishes a fixed preview pose'),
        ('rviz', 'true', 'Open RViz with the saved product view'),
        ('publish_joint_states', 'true', 'Publish preview joint states; false uses an external source'),
        ('rviz_materials', 'true', 'Preserve per-part materials using massless fixed visual frames'),
        ('use_sim_time', 'false', 'Use a simulator /clock topic')]:
        options.append(DeclareLaunchArgument(name, default_value=default, choices=['true', 'false'], description=help_text))
    options.append(DeclareLaunchArgument('rviz_config', default_value=str(share/'config/display.rviz')))
    return LaunchDescription([*options, OpaqueFunction(function=launch_nodes)])
