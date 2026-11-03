"""Validate the imported PAROL6 chain, shared physical data and tool adapters."""

from pathlib import Path
import json
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from parol6_model import (ROOT, ORIGINAL, URDF, MJCF, PROFILES, ARM_JOINTS,
                          ARM_LINKS, PAR6_FROM_PAROL6, SOURCE_FROM_PAROL6,
                          urdf_fk, flange_adapter, source_flange_adapter,
                          ideal_dh, normalized_joint_limits, pose,
                          FOREARM_DISTANCE, SOURCE_FOREARM_DISTANCE,
                          FLANGE_DISTANCE, PAR6_FLANGE_DISTANCE,
                          LOWER_ARM_MESH_OFFSET, wrist_mount_geometry)
from verify_msg_jaws import canonical
from model_colors import load as load_colors, color as part_color, rgba

PAR6_MJCF = ROOT/'mujoco/par6'
PAR6_URDF = ROOT/'urdf/par6_description'


def native_pose(data, name, site=False):
    result = np.eye(4)
    entry = data.site(name) if site else data.body(name)
    result[:3, 3] = entry.xpos
    result[:3, :3] = entry.xmat.reshape(3, 3)
    return result


def pose_error(a, b):
    return (float(np.linalg.norm(a[:3, 3]-b[:3, 3])*1000),
            float(Rotation.from_matrix(a[:3, :3].T@b[:3, :3]).magnitude()*180/np.pi))


def max_error(previous, error):
    return [max(previous[0], error[0]), max(previous[1], error[1])]


def check_shared_properties(native, reference_native, urdf, reference_urdf, profile):
    tool = 'flange_link' if profile == 'flange' else 'pneumatic_gripper_link' if profile == 'pneumatic' else 'msg_gripper_link' if profile.startswith('msg') else 'ssg48_gripper_link'
    bodies = ['base_link', *ARM_LINKS[:5], tool]
    if profile != 'flange':
        bodies += ['left_finger_link', 'right_finger_link']
    for name in bodies:
        for attribute in ['body_mass', 'body_ipos', 'body_iquat', 'body_inertia']:
            np.testing.assert_allclose(getattr(native, attribute)[native.body(name).id],
                                       getattr(reference_native, attribute)[reference_native.body(name).id],
                                       rtol=0, atol=2e-15, err_msg=f'{profile}/{name}/{attribute}')
        a = urdf.find(f'link[@name="{name}"]/inertial')
        b = reference_urdf.find(f'link[@name="{name}"]/inertial')
        assert canonical(a) == canonical(b), f'{profile}/{name}: URDF inertial inheritance changed'
    for name in ARM_JOINTS:
        for attribute in ['dof_armature', 'dof_damping', 'dof_frictionloss']:
            np.testing.assert_array_equal(getattr(native, attribute)[native.joint(name).dofadr],
                                          getattr(reference_native, attribute)[reference_native.joint(name).dofadr])
    for attribute in ['actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange']:
        np.testing.assert_array_equal(getattr(native, attribute), getattr(reference_native, attribute))
    return tool


def verify(poses=160):
    palette = load_colors()
    if poses < 3:
        raise ValueError('At least three poses are required')
    source_path = ORIGINAL/'urdf/parol6.urdf'
    source = ET.parse(source_path).getroot()
    rng = np.random.default_rng(61031)
    low, high = np.array([normalized_joint_limits(source, i) for i in range(1, 7)]).T
    samples = [np.zeros(6), low, high, *[rng.uniform(low, high) for _ in range(poses-3)]]
    report = {'package_version': ET.parse(URDF/'package.xml').findtext('version'),
              'joint_convention': 'PAR6 normalized', 'poses_per_profile': len(samples), 'reference': 'PAR6 arm chain with the user-corrected 177 mm forearm; PAROL6 flange reach corrected to match PAR6 housing-to-tool spacing. Original export retained unchanged.',
              'joint_names': list(ARM_JOINTS), 'source_joint_names': [f'L{i}' for i in range(1, 7)],
              'par6_from_parol6_joint_signs': PAR6_FROM_PAROL6.tolist(),
              'source_from_parol6_joint_signs': SOURCE_FROM_PAROL6.tolist(),
              'forearm_dimension_m': {'par6': .177, 'parol6': FOREARM_DISTANCE},
              'source_forearm_dimension_m': SOURCE_FOREARM_DISTANCE,
              'mounting_geometry': wrist_mount_geometry(),
              'source_l6_from_flange': source_flange_adapter().tolist(),
              'wrist_3_from_flange': flange_adapter().tolist(),
              'profiles': {}, 'ideal_dh_error_max_mm_deg': [0., 0.]}
    source_frames = []
    for q in samples:
        frames = urdf_fk(source_path, dict(zip([f'L{i}' for i in range(1, 7)], q*SOURCE_FROM_PAROL6)))
        source_frames.append(frames)
    settings = __import__('parol6_model').config()
    # Check exact preservation of original triangle records and collision STLs.
    from split_motor_visuals import read_parts
    for source_name, parts in settings['visual_components'].items():
        _, original, _ = read_parts(ORIGINAL/f'meshes/{source_name}.STL')
        emitted = [read_parts(URDF/f'meshes/arm/visual/{part}.STL')[1] for part in parts]
        def records_sorted(records):
            return np.sort(records.view('V50').reshape(-1))
        np.testing.assert_array_equal(records_sorted(original), records_sorted(np.concatenate(emitted)))
        for part in parts:
            assert (URDF/f'meshes/arm/visual/{part}.STL').read_bytes() == (MJCF/f'assets/visual/{part}.STL').read_bytes()
    for name in settings['arm_meshes'].values():
        blob = (ORIGINAL/f'meshes/{name}.STL').read_bytes()
        assert blob == (URDF/f'meshes/arm/{name}.STL').read_bytes() == (MJCF/f'assets/arm/{name}.STL').read_bytes()
    for profile in PROFILES:
        model = mujoco.MjModel.from_xml_path(str(MJCF/f'scene_{profile}.xml'))
        reference = mujoco.MjModel.from_xml_path(str(PAR6_MJCF/f'scene_{profile}.xml'))
        data, prior = mujoco.MjData(model), mujoco.MjData(reference)
        path = URDF/f'parol6_{profile}.urdf'
        urdf = ET.parse(path).getroot()
        reference_urdf = ET.parse(PAR6_URDF/f'par6_{profile}.urdf').getroot()
        tool = check_shared_properties(model, reference, urdf, reference_urdf, profile)
        native_xml = ET.parse(MJCF/f'parol6_{profile}.xml').getroot()
        previous_xml = ET.parse(PAR6_MJCF/f'par6_{profile}.xml').getroot()
        assert canonical(native_xml.find(f'.//body[@name="{tool}"]')) == canonical(previous_xml.find(f'.//body[@name="{tool}"]'))
        for i, name in enumerate(ARM_JOINTS, start=1):
            a = urdf.find(f'joint[@name="{name}"]')
            b = source.find(f'joint[@name="L{i}"]')
            shared = reference_urdf.find(f'joint[@name="{name}"]')
            assert a.get('type') == b.get('type')
            assert canonical(a.find('axis')) == canonical(shared.find('axis'))
            assert a.find('origin').get('rpy') == shared.find('origin').get('rpy')
            expected_pos = [0., 0., -FOREARM_DISTANCE] if i == 5 else np.fromstring(shared.find('origin').get('xyz'), sep=' ')
            np.testing.assert_allclose(np.fromstring(a.find('origin').get('xyz'), sep=' '), expected_pos, rtol=0, atol=1e-15)
            np.testing.assert_allclose([float(a.find('limit').get(key)) for key in ['lower', 'upper']], normalized_joint_limits(source, i), rtol=0, atol=1e-15)
            for key in ['effort', 'velocity']:
                assert a.find('limit').get(key) == b.find('limit').get(key)
            assert bool(model.joint(name).limited[0]) == (b.get('type') != 'continuous')
            np.testing.assert_array_equal(model.joint(name).axis, reference.joint(name).axis)
            np.testing.assert_array_equal(model.body(ARM_LINKS[i-1]).quat, reference.body(ARM_LINKS[i-1]).quat)
        np.testing.assert_allclose(np.fromstring(urdf.find('joint[@name="flange_fixed_joint"]/origin').get('xyz'), sep=' '), [0., 0., FLANGE_DISTANCE], atol=1e-15)
        for shape in urdf.findall('link[@name="wrist_1_link"]/visual')+urdf.findall('link[@name="wrist_1_link"]/collision'):
            np.testing.assert_allclose(np.fromstring(shape.find('origin').get('xyz'), sep=' '), LOWER_ARM_MESH_OFFSET, atol=1e-15)
        for shape in native_xml.findall('.//body[@name="wrist_1_link"]/geom'):
            np.testing.assert_allclose(np.fromstring(shape.get('pos'), sep=' '), LOWER_ARM_MESH_OFFSET, atol=1e-15)
        metrics = {'source_arm_fk_max_mm_deg': [0., 0.], 'corrected_source_arm_fk_max_mm_deg': [0., 0.], 'source_flange_fk_max_mm_deg': [0., 0.],
                   'urdf_native_tcp_max_mm_deg': [0., 0.], 'mapped_par6_flange_max_mm_deg': [0., 0.],
                   'mapped_par6_flange_residual_max_mm_deg': [0., 0.],
                   'mapped_par6_mass_matrix_max_relative_difference': 0., 'mapped_par6_gravity_max_difference_nm': 0.,
                   'shared_mass_com_inertia_and_armature_checked': True, 'shared_tool_subtree_identical': True,
                   'par6_axes_and_origin_orientations_identical': True, 'canonical_chain_residual_max_mm_deg': [0., 0.]}
        signs = np.r_[PAR6_FROM_PAROL6, np.ones(model.nv-6)]
        for q, imported in zip(samples, source_frames):
            data.qpos[:6] = q
            prior.qpos[:6] = q*PAR6_FROM_PAROL6
            mujoco.mj_forward(model, data)
            mujoco.mj_forward(reference, prior)
            poses_urdf = urdf_fk(path, dict(zip(ARM_JOINTS, q)))
            shared_poses = urdf_fk(PAR6_URDF/f'par6_{profile}.urdf', dict(zip(ARM_JOINTS, q)))
            source_delta = imported['L4'][:3, :3]@LOWER_ARM_MESH_OFFSET
            for i, body in enumerate(ARM_LINKS, start=1):
                imported_pose = imported[f'L{i}'] if i < 6 else imported['L6']@pose(rpy=(0., np.pi, 0.))
                for actual in [native_pose(data, body), poses_urdf[body]]:
                    metrics['source_arm_fk_max_mm_deg'] = max_error(metrics['source_arm_fk_max_mm_deg'], pose_error(imported_pose, actual))
                    corrected = imported_pose.copy()
                    if i >= 5:
                        corrected[:3, 3] += source_delta
                    metrics['corrected_source_arm_fk_max_mm_deg'] = max_error(metrics['corrected_source_arm_fk_max_mm_deg'], pose_error(corrected, actual))
                expected_native = native_pose(prior, body)
                expected_urdf = shared_poses[body].copy()
                for actual, expected_frame in [(native_pose(data, body), expected_native), (poses_urdf[body], expected_urdf)]:
                    metrics['canonical_chain_residual_max_mm_deg'] = max_error(metrics['canonical_chain_residual_max_mm_deg'], pose_error(expected_frame, actual))
            expected = imported['L6']@source_flange_adapter()
            expected[:3, 3] += source_delta
            for actual in [native_pose(data, 'flange'), poses_urdf['flange']]:
                metrics['source_flange_fk_max_mm_deg'] = max_error(metrics['source_flange_fk_max_mm_deg'], pose_error(expected, actual))
                report['ideal_dh_error_max_mm_deg'] = max_error(report['ideal_dh_error_max_mm_deg'], pose_error(actual, ideal_dh(q)[-1]))
            metrics['urdf_native_tcp_max_mm_deg'] = max_error(metrics['urdf_native_tcp_max_mm_deg'], pose_error(poses_urdf['tcp'], native_pose(data, 'tcp', site=True)))
            metrics['mapped_par6_flange_max_mm_deg'] = max_error(metrics['mapped_par6_flange_max_mm_deg'], pose_error(native_pose(data, 'flange'), native_pose(prior, 'flange')))
            flange_expected = native_pose(prior, 'flange')
            flange_expected[:3, 3] += native_pose(prior, 'wrist_3_link')[:3, :3]@np.array([0., 0., FLANGE_DISTANCE-PAR6_FLANGE_DISTANCE])
            metrics['mapped_par6_flange_residual_max_mm_deg'] = max_error(metrics['mapped_par6_flange_residual_max_mm_deg'], pose_error(native_pose(data, 'flange'), flange_expected))
            matrix = np.empty((model.nv, model.nv));reference_matrix = np.empty_like(matrix)
            mujoco.mj_fullM(model, matrix, data.qM)
            mujoco.mj_fullM(reference, reference_matrix, prior.qM)
            expected_matrix = signs[:, None]*reference_matrix*signs[None, :]
            relative = float(np.linalg.norm(matrix-expected_matrix)/np.linalg.norm(expected_matrix))
            metrics['mapped_par6_mass_matrix_max_relative_difference'] = max(metrics['mapped_par6_mass_matrix_max_relative_difference'], relative)
            gravity = float(np.max(np.abs(data.qfrc_bias-signs*prior.qfrc_bias)))
            metrics['mapped_par6_gravity_max_difference_nm'] = max(metrics['mapped_par6_gravity_max_difference_nm'], gravity)
            assert np.linalg.eigvalsh(matrix).min() > 0
        assert metrics['canonical_chain_residual_max_mm_deg'][0] < 1e-8, metrics
        assert metrics['canonical_chain_residual_max_mm_deg'][1] < 1e-8, metrics
        assert metrics['source_arm_fk_max_mm_deg'][0] < .67, metrics
        assert metrics['source_arm_fk_max_mm_deg'][1] < .003, metrics
        assert metrics['corrected_source_arm_fk_max_mm_deg'][0] < .02, metrics
        assert metrics['corrected_source_arm_fk_max_mm_deg'][1] < .003, metrics
        assert metrics['source_flange_fk_max_mm_deg'][0] < .02, metrics
        assert metrics['source_flange_fk_max_mm_deg'][1] < .003, metrics
        assert metrics['urdf_native_tcp_max_mm_deg'][0] < .02, metrics
        assert metrics['urdf_native_tcp_max_mm_deg'][1] < .01, metrics
        assert abs(metrics['mapped_par6_flange_max_mm_deg'][0]-abs(FLANGE_DISTANCE-PAR6_FLANGE_DISTANCE)*1000) < 1e-8, metrics
        assert metrics['mapped_par6_flange_max_mm_deg'][1] < .01, metrics
        assert metrics['mapped_par6_flange_residual_max_mm_deg'][0] < 1e-8, metrics
        assert metrics['mapped_par6_flange_residual_max_mm_deg'][1] < 1e-8, metrics
        for part in ['j1_actuator_visual', 'j2_actuator_visual', 'j3_actuator_visual']:
            geom = model.geom(part)
            assert geom.contype[0] == geom.conaffinity[0] == 0
            color = model.mat_rgba[geom.matid[0]]
            expected = rgba(part_color(palette, part[:2]+'_motor'))
            np.testing.assert_allclose(color, expected, rtol=0, atol=3e-8)
        if profile != 'flange':
            for side in ['left', 'right']:
                link = urdf.find(f'link[@name="{side}_finger_link"]')
                count = len(link.findall('collision'))
                assert count in range(1, 7) if profile == 'pneumatic' else count == 6
        report['profiles'][profile] = metrics
        print(profile, 'PAR6 joint frames and shared properties/tools checked;',
              'PAR6 flange delta mm/deg', metrics['mapped_par6_flange_max_mm_deg'], flush=True)
    report['visual_triangle_records_preserved'] = True
    report['original_arm_collision_meshes_preserved'] = True
    assert report['ideal_dh_error_max_mm_deg'][0] < .02
    assert report['ideal_dh_error_max_mm_deg'][1] < .003
    (ROOT/'reports/kinematics/parol6_validation.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    from build_kinematics_docs import build as docs
    docs(report)
    return report


if __name__ == '__main__':
    verify()
