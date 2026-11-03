# SPDX-License-Identifier: Apache-2.0
"""Assemble an installed MoveIt demo without changing the description package."""
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ARM_JOINTS = ('base_joint', 'shoulder_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def description_helper(description_share):
    spec = importlib.util.spec_from_file_location('source_robot_description',
        Path(description_share)/'scripts/robot_description.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def select_profile(description_share, family, tool='msg', jaws='default'):
    helper = description_helper(description_share)
    entry = helper.select_model(family, tool, jaws)
    return entry.removeprefix(family+'_').removesuffix('.urdf')


def assemble_description(description_share, family, profile, data, limits, *,
                         rviz_materials=True, distro='jazzy'):
    """Apply explicit planning limits and add only mock ros2_control hardware.

    Saved meshes, joint origins/axes, inertia and collisions are never edited.
    The planning instance bounds historical continuous joints so the planner
    respects the captured software ranges instead of wrapping through a stop.
    """
    helper = description_helper(description_share)
    root = ET.fromstring(helper.load_description(Path(description_share)/f'{family}_{profile}.urdf',
                         family+'_description', rviz_materials=rviz_materials))
    planning_limits = {}
    for name, bounds in data['position_limits'].items():
        joint = root.find(f'joint[@name="{name}"]')
        if joint is None:
            raise ValueError(f'Planning configuration references missing joint: {name}')
        settings = limits['arm' if name in ARM_JOINTS else 'gripper']
        if not all(math.isfinite(float(settings[k])) and float(settings[k]) > 0
                   for k in ('max_velocity', 'max_acceleration', 'effort')):
            raise ValueError(f'{name}: demo speed, acceleration and effort must be finite and positive')
        if joint.get('type') == 'continuous':
            joint.set('type', 'revolute')
        limit = joint.find('limit')
        for key, value in dict(lower=bounds[0], upper=bounds[1],
                               velocity=settings['max_velocity'], effort=settings['effort']).items():
            limit.set(key, str(value))
        planning_limits[name] = dict(has_position_limits=True, min_position=bounds[0], max_position=bounds[1],
            has_velocity_limits=True, max_velocity=float(settings['max_velocity']),
            has_acceleration_limits=True, max_acceleration=float(settings['max_acceleration']))
    control = ET.SubElement(root, 'ros2_control', name='SourceRoboticsMockSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mock_components/GenericSystem'
    ET.SubElement(hardware, 'param', name='calculate_dynamics').text = 'false'
    for name in data['position_limits']:
        original = root.find(f'joint[@name="{name}"]')
        joint = ET.SubElement(control, 'joint', name=name)
        mimic = original.find('mimic')
        if mimic is None:
            ET.SubElement(joint, 'command_interface', name='position')
        elif distro == 'humble':
            # Humble's GenericSystem reads per-joint parameters; newer versions
            # get the mimic relation directly from the URDF parser.
            ET.SubElement(joint, 'param', name='mimic').text = mimic.get('joint')
            ET.SubElement(joint, 'param', name='multiplier').text = mimic.get('multiplier', '1')
        state = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(data['initial_positions'][name])
        ET.SubElement(joint, 'state_interface', name='velocity')
    return ET.tostring(root, encoding='unicode'), planning_limits


def controller_configuration(data, update_rate=100):
    groups = {'arm_controller': list(ARM_JOINTS)}
    if data['has_gripper']:
        groups['gripper_controller'] = ['left_finger_joint']
    manager = {'update_rate': update_rate,
               'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'}}
    control = {'controller_manager': {'ros__parameters': manager}}
    moveit = {'controller_names': list(groups)}
    for name, joints in groups.items():
        manager[name] = {'type': 'joint_trajectory_controller/JointTrajectoryController'}
        control[name] = {'ros__parameters': dict(joints=joints, command_interfaces=['position'],
            state_interfaces=['position', 'velocity'], allow_partial_joints_goal=False,
            constraints={'goal_time': 2.0, 'stopped_velocity_tolerance': 0.02})}
        moveit[name] = dict(type='FollowJointTrajectory', action_ns='follow_joint_trajectory',
                            default=True, joints=joints)
    return control, {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                     'moveit_simple_controller_manager': moveit}
