# SPDX-License-Identifier: Apache-2.0
"""Installed, tool-selectable MoveIt planning and mock execution demo."""
import importlib.util
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

FAMILY = 'par6'


def launch_nodes(context):
    share = Path(get_package_share_directory(FAMILY+'_moveit_config'))
    description_share = Path(get_package_share_directory(FAMILY+'_description'))
    spec = importlib.util.spec_from_file_location('source_moveit_model', share/'scripts/model_config.py')
    helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    value = lambda name: LaunchConfiguration(name).perform(context)
    profile = helper.select_profile(description_share, FAMILY, value('end_effector'), value('jaws'))
    data = json.loads((share/f'config/profiles/{profile}.json').read_text())
    load = lambda filename: yaml.safe_load((share/'config'/filename).read_text())
    description, limits = helper.assemble_description(description_share, FAMILY, profile, data,
        load('joint_limits.yaml'), rviz_materials=value('rviz_materials') == 'true',
        distro=os.environ.get('ROS_DISTRO', 'jazzy'))
    control, controllers = helper.controller_configuration(data)
    config = dict(robot_description=ParameterValue(description, value_type=str),
        robot_description_semantic=ParameterValue((share/f'config/profiles/{profile}.srdf').read_text(), value_type=str),
        robot_description_kinematics=load('kinematics.yaml'),
        robot_description_planning={'joint_limits': limits},
        planning_pipelines=['ompl'], default_planning_pipeline='ompl',
        ompl=load('ompl_humble.yaml' if os.environ.get('ROS_DISTRO') == 'humble' else 'ompl_planning.yaml'),
        use_sim_time=False,
        **controllers,
        **load('planning_parameters.yaml'))
    # Controller YAML contains settings scoped to several ROS nodes.
    import tempfile
    # NamedTemporaryFile is closed before controller_manager opens it. The
    # launch shutdown handler removes it on normal shutdown and on failure.
    fd, filename = tempfile.mkstemp(prefix=FAMILY+'_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as handle:
        yaml.safe_dump(control, handle)
    def cleanup(_context):
        Path(filename).unlink(missing_ok=True)
        return []
    broadcaster = Node(package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'], output='screen')
    arm = Node(package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager-timeout', '60'], output='screen')
    gripper = Node(package='controller_manager', executable='spawner',
        arguments=['gripper_controller', '--controller-manager-timeout', '60'], output='screen') if data['has_gripper'] else None
    planning = [Node(package='moveit_ros_move_group', executable='move_group', parameters=[config], output='screen')]
    if value('rviz') == 'true':
        planning.append(Node(package='rviz2', executable='rviz2', name='moveit_rviz',
            arguments=['-d', value('rviz_config')], parameters=[config], output='screen'))
    def after_success(next_actions):
        def callback(event, _context):
            if event.returncode != 0:
                return [LogInfo(msg='Controller activation failed; stopping the mock demo.'),
                        EmitEvent(event=Shutdown(reason='Controller activation failed'))]
            return next_actions
        return callback
    sequence = [broadcaster, arm]+([gripper] if gripper else [])
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=after_success([sequence[index+1]] if index+1<len(sequence) else planning)))
        for index,node in enumerate(sequence)]
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])), *handlers,
        LogInfo(msg=f'{FAMILY.upper()}: MoveIt {profile}; mock hardware only; TCP=tcp'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': config['robot_description']}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='world_to_base',
             arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': config['robot_description']}, filename],
             remappings=[('~/robot_description', '/robot_description')], output='screen'),
        broadcaster]
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory(FAMILY+'_moveit_config'))
    return LaunchDescription([
        DeclareLaunchArgument('end_effector', default_value='msg',
            choices=['flange', 'msg', 'ssg48', 'pneumatic', 'msg_tpu_glue', 'ssg48_stock']),
        DeclareLaunchArgument('jaws', default_value='default', choices=['default', 'solid', 'soft_tpu', 'fin_ray']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_materials', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=str(share/'config/moveit.rviz')),
        OpaqueFunction(function=launch_nodes)])
