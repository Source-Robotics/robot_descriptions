import copy
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from model_reference import ROOT, REFERENCE, RCB_FROM_MJ_OFFSET
from build_moveit_support import PROFILES, profile_data, rigid_collision_pairs, semantic
from moveit_support.model_config import ARM_JOINTS, assemble_description, controller_configuration, select_profile
from ros_support.robot_description import load_description

DEMO_LIMITS = {'arm': dict(max_velocity=.5, max_acceleration=1., effort=300.),
               'gripper': dict(max_velocity=.03, max_acceleration=.1, effort=50.)}


@pytest.mark.parametrize('family', ['par6', 'parol6'])
@pytest.mark.parametrize('profile', PROFILES)
def test_moveit_adapter_preserves_all_physical_data_and_bounds_every_actuated_joint(family, profile):
    share = ROOT/f'urdf/{family}_description'
    root = ET.parse(share/f'{family}_{profile}.urdf').getroot()
    data = profile_data(family, root)
    before = ET.fromstring(load_description(share/f'{family}_{profile}.urdf', share.name))
    text, limits = assemble_description(share, family, profile, data, DEMO_LIMITS, rviz_materials=False)
    after = ET.fromstring(text)
    control = after.find('ros2_control')
    assert control.find('hardware/plugin').text == 'mock_components/GenericSystem'
    after.remove(control)
    for old, new in zip(before.findall('joint'), after.findall('joint')):
        assert old.get('name') == new.get('name')
        if old.get('type') == 'fixed':continue
        assert new.get('type') in ['revolute', 'prismatic']
        limit = new.find('limit')
        bounds = data['position_limits'][new.get('name')]
        assert [float(limit.get(k)) for k in ['lower', 'upper']] == bounds
        assert bounds[0] <= data['initial_positions'][new.get('name')] <= bounds[1]
        assert limits[new.get('name')]['has_acceleration_limits']
        new.set('type', old.get('type'))
        new.remove(limit); new.insert(list(old).index(old.find('limit')), copy.deepcopy(old.find('limit')))
    # Whole-tree equality covers inertia/COM, all visual/collision geometry,
    # every frame, joint axis, camera frame, dynamics and mimic relationship.
    assert ET.tostring(before) == ET.tostring(after)
    if family == 'par6':
        expected = np.asarray(REFERENCE['variants']['flange']['software_joint_limits_rad'])-RCB_FROM_MJ_OFFSET
        np.testing.assert_allclose(np.asarray([data['position_limits'][n] for n in ARM_JOINTS]).T, expected, atol=1e-15)
    else:
        for name in ARM_JOINTS:
            old = root.find(f'joint[@name="{name}"]/limit')
            assert data['position_limits'][name] == [float(old.get('lower')),float(old.get('upper'))]
    controller, moveit = controller_configuration(data)
    names = moveit['moveit_simple_controller_manager']['controller_names']
    assert ('gripper_controller' in names) == data['has_gripper']
    assert controller['arm_controller']['ros__parameters']['joints'] == list(ARM_JOINTS)
    if data['has_gripper']:
        assert controller['gripper_controller']['ros__parameters']['joints'] == ['left_finger_joint']
        right = control.find('joint[@name="right_finger_joint"]')
        assert right.find('command_interface') is None


@pytest.mark.parametrize('family', ['par6', 'parol6'])
@pytest.mark.parametrize('profile', PROFILES)
def test_semantics_use_selected_tcp_and_leave_nonadjacent_collisions_enabled(family, profile):
    root = ET.parse(ROOT/f'urdf/{family}_description/{family}_{profile}.urdf').getroot()
    data = profile_data(family, root); srdf = ET.fromstring(semantic(root, data))
    assert srdf.get('name') == root.get('name')
    assert srdf.find('group[@name="arm"]/chain').attrib == {'base_link':'base_link', 'tip_link':'tcp'}
    disabled = {frozenset([a,b]) for a,b,_ in rigid_collision_pairs(root)}
    assert frozenset(['left_finger_link','right_finger_link']) not in disabled
    assert frozenset(['base_link','forearm_link']) not in disabled
    assert frozenset(['upper_arm_link','wrist_2_link']) not in disabled
    assert frozenset(['base_link','shoulder_link']) in disabled
    for state in srdf.findall('group_state'):
        for joint in state.findall('joint'):
            low, high = data['position_limits'][joint.get('name')]
            assert low <= float(joint.get('value')) <= high
    if data['has_gripper']:
        assert srdf.find('end_effector').get('parent_link') == 'tcp'
        assert srdf.find('group_state[@name="open"]') is not None
        assert srdf.find('group_state[@name="closed"]') is not None
    if profile == 'pneumatic':
        assert data['named_states']['open']['left_finger_joint'] == 0.
        assert data['named_states']['closed']['left_finger_joint'] == .0035


@pytest.mark.parametrize('distro', ['humble', 'jazzy'])
def test_mock_mimic_joint_is_never_independently_commanded(distro):
    share = ROOT/'urdf/par6_description'; root = ET.parse(share/'par6_msg.urdf').getroot()
    text,_ = assemble_description(share, 'par6', 'msg', profile_data('par6',root), DEMO_LIMITS, distro=distro)
    right = ET.fromstring(text).find('ros2_control/joint[@name="right_finger_joint"]')
    assert right.find('command_interface') is None
    if distro == 'humble':
        assert right.find('param[@name="mimic"]').text == 'left_finger_joint'
        assert right.find('param[@name="multiplier"]').text == '-1'


def test_selection_reuses_description_compatibility_checks():
    share = ROOT/'urdf/par6_description'
    assert select_profile(share, 'par6', 'msg', 'soft_tpu') == 'msg_tpu_glue'
    with pytest.raises(ValueError, match='jaws must be'):
        select_profile(share, 'par6', 'msg', 'fin_ray')


@pytest.mark.parametrize('key,value', [('max_velocity',0),('max_acceleration',float('nan')),('effort',-1)])
def test_invalid_demo_rates_fail_before_building_a_planning_description(key,value):
    share=ROOT/'urdf/par6_description';root=ET.parse(share/'par6_msg.urdf').getroot()
    limits=copy.deepcopy(DEMO_LIMITS);limits['arm'][key]=value
    with pytest.raises(ValueError,match='finite and positive'):
        assemble_description(share,'par6','msg',profile_data('par6',root),limits)
