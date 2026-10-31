"""Frame-explicit helpers for the imported PAROL6 URDF and shared PAR6 tools."""

from pathlib import Path
import json
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'source/parol6'
ORIGINAL = SOURCE / 'original'
URDF = ROOT / 'urdf/parol6_description'
MJCF = ROOT / 'mujoco/parol6'
PROFILES = ('flange', 'msg', 'msg_tpu_glue', 'ssg48', 'ssg48_stock', 'pneumatic')
ARM_JOINTS = ('base_joint', 'shoulder_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
ARM_LINKS = ('shoulder_link', 'upper_arm_link', 'forearm_link',
             'wrist_1_link', 'wrist_2_link', 'wrist_3_link')
# Public PAROL6 coordinates now match PAR6. Only the preserved, historical
# SolidWorks export uses these reversed signs for joints 3 through 6.
SOURCE_FROM_PAROL6 = np.array([1., 1., -1., -1., -1., -1.])
PAR6_FROM_PAROL6 = np.ones(6)
def config():
    return json.loads((SOURCE / 'model.json').read_text(encoding='utf-8'))


GEOMETRY = config()['geometry']
FOREARM_DISTANCE = GEOMETRY['forearm_distance_m']
SOURCE_FOREARM_DISTANCE = GEOMETRY['source_forearm_distance_m']
FLANGE_DISTANCE = GEOMETRY['flange_distance_m']
PAR6_FLANGE_DISTANCE = GEOMETRY['par6_flange_distance_m']
LOWER_ARM_MESH_OFFSET = np.array([0., 0., SOURCE_FOREARM_DISTANCE-FOREARM_DISTANCE])


def wrist_mount_geometry():
    """Measure housing faces, excluding the PAROL6 motor's protruding shaft."""
    from split_motor_visuals import read_parts
    reference = GEOMETRY['wrist_housing_reference']
    fronts, measured = {}, {}
    for robot in ['par6', 'parol6']:
        path = ROOT/reference[f'{robot}_mesh']
        _, records, groups = read_parts(path)
        vertices = records['vertex'][groups[reference['housing_component']]].reshape(-1, 3)
        measured[robot] = float(np.max(vertices[:, 1]))
        fronts[robot] = reference[f'{robot}_housing_front_m']
        if abs(measured[robot]-fronts[robot]) > 2e-8:
            raise ValueError(f'{robot}: wrist housing face changed; review the flange reference')
    expected = PAR6_FLANGE_DISTANCE+fronts['parol6']-fronts['par6']
    if abs(expected-FLANGE_DISTANCE) > 1e-12:
        raise ValueError('PAROL6 flange offset no longer matches PAR6 housing-to-tool spacing')
    return {'housing_front_from_j6_m': fronts, 'measured_housing_front_from_j6_m': measured,
            'flange_distance_m': {'par6': PAR6_FLANGE_DISTANCE, 'parol6': FLANGE_DISTANCE},
            'flange_shift_from_par6_m': FLANGE_DISTANCE-PAR6_FLANGE_DISTANCE,
            'lower_arm_mesh_offset_m': LOWER_ARM_MESH_OFFSET.tolist(),
            'basis': reference['method']}


def pose(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    transform = np.eye(4)
    transform[:3, 3] = xyz
    transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    return transform


def origin(element):
    if element is None:
        return np.eye(4)
    return pose(np.fromstring(element.get('xyz', '0 0 0'), sep=' '),
                np.fromstring(element.get('rpy', '0 0 0'), sep=' '))


def urdf_fk(path, values=None):
    """Evaluate actual URDF origins/axes, including fixed and prismatic joints."""
    values = {} if values is None else values
    root = ET.parse(path).getroot()
    children = {j.find('child').get('link') for j in root.findall('joint')}
    roots = {link.get('name') for link in root.findall('link')} - children
    if len(roots) != 1:
        raise ValueError('Expected one URDF root')
    frames = {roots.pop(): np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            motion = np.eye(4)
            if joint.get('type') != 'fixed':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                value = values.get(joint.get('name'), 0.)
                if joint.get('type') == 'prismatic':
                    motion[:3, 3] = axis * value
                else:
                    motion[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
            frames[joint.find('child').get('link')] = frames[parent] @ origin(joint.find('origin')) @ motion
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError('Disconnected/cyclic URDF joint tree')
    return frames


def flange_adapter():
    """Normalized wrist_3_link -> PAROL6 flange, with matching housing spacing."""
    return pose((0., 0., FLANGE_DISTANCE))


def source_flange_adapter():
    """Original export's L6 -> common flange, for source comparisons only."""
    return pose((0., 0., -FLANGE_DISTANCE), (0., np.pi, 0.))


def normalized_joint_limits(source, index):
    limit = source.find(f'joint[@name="L{index}"]/limit')
    low, high = float(limit.get('lower')), float(limit.get('upper'))
    return (low, high) if SOURCE_FROM_PAROL6[index-1] > 0 else (-high, -low)


def ideal_dh(q, robot='parol6'):
    """Standard DH approximation at the common flange, using package angles.

    Generated PAROL6 now uses PAR6 axes and joint-origin orientations. The
    original export's rounded RPY values are preserved only as source data.
    """
    q = np.asarray(q, dtype=float)
    if robot == 'parol6':
        forearm = FOREARM_DISTANCE
        flange = FLANGE_DISTANCE
    elif robot == 'par6':
        forearm = .177
        flange = PAR6_FLANGE_DISTANCE
    else:
        raise ValueError(robot)
    theta = q + np.array([0., -np.pi/2, np.pi, 0., 0., np.pi])
    alpha = [-np.pi/2, np.pi, np.pi/2, -np.pi/2, np.pi/2, np.pi]
    a = [.0234207210610375, .18, -.0435, 0., 0., 0.]
    d = [.1105, 0., 0., -forearm, 0., -flange]
    frames = [np.eye(4)]
    for th, al, length, offset in zip(theta, alpha, a, d):
        c, s, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
        step = np.array([[c, -s*ca, s*sa, length*c],
                         [s, c*ca, -c*sa, length*s],
                         [0., sa, ca, offset], [0., 0., 0., 1.]])
        frames.append(frames[-1] @ step)
    return np.asarray(frames)
