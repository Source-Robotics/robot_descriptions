import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from parol6_model import (ROOT, ORIGINAL, URDF, MJCF, PROFILES, ARM_JOINTS,
                          ARM_LINKS, urdf_fk, ideal_dh, origin,
                          LOWER_ARM_MESH_OFFSET, FLANGE_DISTANCE,
                          PAR6_FLANGE_DISTANCE, wrist_mount_geometry)
from verify_parol6 import native_pose, check_shared_properties
from verify_msg_jaws import canonical
from ros_urdf import convert


@pytest.mark.parametrize('profile', PROFILES)
def test_arm_frames_match_par6_and_tools_follow_shorter_wrist(profile):
    q = np.array([.2, -.3, .6, -.4, .7, -1.2])
    shared = urdf_fk(ROOT/f'urdf/par6_description/par6_{profile}.urdf', dict(zip(ARM_JOINTS, q)))
    exported = urdf_fk(URDF/f'parol6_{profile}.urdf', dict(zip(ARM_JOINTS, q)))
    model = mujoco.MjModel.from_xml_path(str(MJCF/f'scene_{profile}.xml'))
    data = mujoco.MjData(model)
    data.qpos[:6] = q
    mujoco.mj_forward(model, data)
    reference = mujoco.MjModel.from_xml_path(str(ROOT/f'mujoco/par6/scene_{profile}.xml'))
    previous = mujoco.MjData(reference);previous.qpos[:6] = q
    mujoco.mj_forward(reference, previous)
    delta_urdf = shared['wrist_3_link'][:3, :3]@np.array([0., 0., -.0066])
    delta_native = native_pose(previous, 'wrist_3_link')[:3, :3]@np.array([0., 0., -.0066])
    for i, link in enumerate(ARM_LINKS, start=1):
        expected_urdf = shared[link].copy();expected_native = native_pose(previous, link)
        np.testing.assert_allclose(exported[link], expected_urdf, rtol=0, atol=2e-14)
        np.testing.assert_allclose(native_pose(data, link), expected_native, rtol=0, atol=2e-14)
        np.testing.assert_array_equal(model.joint(ARM_JOINTS[i-1]).axis, reference.joint(ARM_JOINTS[i-1]).axis)
    expected = shared['flange'].copy();expected[:3, 3] += delta_urdf
    np.testing.assert_allclose(exported['flange'], expected, rtol=0, atol=2e-14)
    # Flange is the common parent: cameras, fingers, TCPs and tool inertials
    # must translate together, keeping their own mounting transforms intact.
    tool_frames = ['tcp', 'flange_link'] if profile == 'flange' else ['tcp', 'left_finger_link', 'right_finger_link', 'msg_camera_link'] if profile.startswith('msg') else ['tcp', 'left_finger_link', 'right_finger_link']
    for name in tool_frames:
        expected = shared[name].copy(); expected[:3, 3] += delta_urdf
        np.testing.assert_allclose(exported[name], expected, rtol=0, atol=2e-14)
        if name == 'msg_camera_link':
            ci, pi = model.camera('msg_camera').id, reference.camera('msg_camera').id
            np.testing.assert_allclose(data.cam_xpos[ci], previous.cam_xpos[pi]+delta_native, atol=2e-14)
            np.testing.assert_allclose(data.cam_xmat[ci], previous.cam_xmat[pi], atol=2e-14)
            continue
        native = native_pose(data, name, site=name == 'tcp')
        expected_native = native_pose(previous, name, site=name == 'tcp')
        expected_native[:3, 3] += delta_native
        np.testing.assert_allclose(native, expected_native, rtol=0, atol=2e-14)


def test_lower_arm_visual_and_collision_follow_corrected_forearm():
    path = URDF/'parol6_flange.urdf'
    tree = ET.parse(path)
    joint = tree.find('joint[@name="wrist_2_joint"]/origin')
    np.testing.assert_allclose(origin(joint)[:3, 3], [0, 0, -.177], atol=1e-15)
    shapes = tree.findall('link[@name="wrist_1_link"]/visual')+tree.findall('link[@name="wrist_1_link"]/collision')
    for shape in shapes:
        np.testing.assert_allclose(origin(shape.find('origin'))[:3, 3], [0, 0, -.00065], atol=1e-15)
    root = ET.parse(MJCF/'parol6_flange.xml')
    for geom in root.findall('.//body[@name="wrist_1_link"]/geom'):
        np.testing.assert_allclose(np.fromstring(geom.get('pos'), sep=' '), [0, 0, -.00065], atol=1e-15)


def test_flange_rear_face_clearance_matches_actual_wrist_housings():
    import trimesh
    measured = wrist_mount_geometry()
    # Evaluate the shared flange's actual mesh and fixed mounting transform,
    # instead of assuming a frame is located on its rear surface.
    gaps, native_gaps = [], []
    for robot in ['par6', 'parol6']:
        path = ROOT/f'urdf/{robot}_description/{robot}_flange.urdf'
        frames = urdf_fk(path)
        shape = ET.parse(path).find('link[@name="flange_link"]/visual')
        mesh = trimesh.load(path.parent/shape.find('geometry/mesh').get('filename'), force='mesh')
        transform = np.linalg.inv(frames['wrist_3_link'])@frames['flange_link']@origin(shape.find('origin'))
        points = mesh.vertices@transform[:3, :3].T+transform[:3, 3]
        rear = points[:, 2].min()
        gaps.append(rear-measured['housing_front_from_j6_m'][robot])
        model = mujoco.MjModel.from_xml_path(str(ROOT/f'mujoco/{robot}/scene_flange.xml'))
        data = mujoco.MjData(model); mujoco.mj_forward(model, data)
        geom = int(model.body_geomadr[model.body('flange_link').id])
        mesh_id = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        vertices = model.mesh_vert[start:start+count]
        world = vertices@data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom]
        wrist = native_pose(data, 'wrist_3_link')
        axial = (world-wrist[:3, 3])@wrist[:3, 2]
        native_gaps.append(axial.min()-measured['housing_front_from_j6_m'][robot])
    np.testing.assert_allclose(gaps, [.001, .001], atol=2e-8)
    np.testing.assert_allclose(native_gaps, [.001, .001], atol=2e-8)
    assert gaps[0] == pytest.approx(gaps[1], abs=1e-12)


@pytest.mark.parametrize('profile', PROFILES)
def test_physical_properties_and_complete_native_tool_match_par6(profile):
    model = mujoco.MjModel.from_xml_path(str(MJCF/f'parol6_{profile}.xml'))
    reference = mujoco.MjModel.from_xml_path(str(ROOT/f'mujoco/par6/par6_{profile}.xml'))
    urdf = ET.parse(URDF/f'parol6_{profile}.urdf').getroot()
    original = ET.parse(ROOT/f'urdf/par6_description/par6_{profile}.urdf').getroot()
    tool = check_shared_properties(model, reference, urdf, original, profile)
    native = ET.parse(MJCF/f'parol6_{profile}.xml').getroot()
    prior = ET.parse(ROOT/f'mujoco/par6/par6_{profile}.xml').getroot()
    assert canonical(native.find(f'.//body[@name="{tool}"]')) == canonical(prior.find(f'.//body[@name="{tool}"]'))


def test_standard_dh_matches_normalized_urdf_flange():
    for q in [np.zeros(6), np.array([.2, -.3, .6, -.4, .7, -1.2])]:
        normalized = urdf_fk(URDF/'parol6_flange.urdf', dict(zip(ARM_JOINTS, q)))
        np.testing.assert_allclose(ideal_dh(q)[-1], normalized['flange'], rtol=0, atol=2e-14)


def test_old_elbow_bounds_are_converted_into_the_new_coordinate_direction():
    joint = ET.parse(URDF/'parol6_flange.urdf').find('joint[@name="elbow_joint"]/limit')
    np.testing.assert_array_equal([float(joint.get('lower')), float(joint.get('upper'))], [-1.3, 2.])


def test_ros_mesh_resolution_does_not_rewrite_portable_files():
    path = URDF/'parol6_msg.urdf'
    before = path.read_bytes()
    resolved = ET.fromstring(convert(path, 'parol6_description'))
    assert all(mesh.get('filename').startswith('package://parol6_description/meshes/') for mesh in resolved.findall('.//mesh'))
    assert path.read_bytes() == before
    for name in ['launch/display.launch', 'launch/gazebo.launch']:
        ET.parse(URDF/name)
