import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import trimesh

from build_pneumatic import CONFIG, PARTS
from parol6_model import ROOT, ARM_JOINTS, urdf_fk, origin
from split_motor_visuals import read_parts


def test_pneumatic_visual_partition_preserves_every_source_triangle():
    config = json.loads(CONFIG.read_text())
    original = read_parts(ROOT/config['source_mesh'])[1]
    parts = [read_parts(ROOT/f'mujoco/par6/assets/pneumatic/{name}.STL')[1] for name in PARTS]
    np.testing.assert_array_equal(np.sort(original.view('V50').ravel()), np.sort(np.concatenate(parts).view('V50').ravel()))
    for name in PARTS:
        assert (ROOT/f'mujoco/par6/assets/pneumatic/{name}.STL').read_bytes() == (ROOT/f'urdf/par6_description/meshes/pneumatic/{name}.STL').read_bytes()


@pytest.mark.parametrize('robot', ['par6', 'parol6'])
def test_pneumatic_jaws_close_symmetrically_and_tcp_stays_fixed(robot):
    path = ROOT/f'urdf/{robot}_description/{robot}_pneumatic.urdf'
    model = mujoco.MjModel.from_xml_path(str(ROOT/f'mujoco/{robot}/scene_pneumatic.xml'))
    data = mujoco.MjData(model)
    tree = ET.parse(path)
    q = [.23, -.32, .47, -.51, .66, -.71]
    data.qpos[:6] = q
    openings, tcp = [], []
    for position in [0., .0035]:
        values = dict(zip(ARM_JOINTS, q)); values.update(left_finger_joint=position, right_finger_joint=position)
        frames = urdf_fk(path, values)
        data.qpos[6:] = position; mujoco.mj_forward(model, data)
        housing = frames['pneumatic_gripper_link']
        boundaries = []
        for side in ['left', 'right']:
            name = side+'_finger_link'
            visual = tree.find(f'link[@name="{name}"]/visual')
            mesh = trimesh.load(path.parent/visual.find('geometry/mesh').get('filename'), force='mesh')
            transform = np.linalg.inv(housing)@frames[name]@origin(visual.find('origin'))
            v = mesh.vertices@transform[:3, :3].T+transform[:3, 3]
            boundaries.append(v[:, 1].min() if side == 'left' else v[:, 1].max())
            np.testing.assert_allclose(data.body(name).xpos, frames[name][:3, 3], atol=4e-6)
        openings.append(boundaries[0]-boundaries[1]); tcp.append(frames['tcp'])
    np.testing.assert_allclose(openings, [.0071, .0001], atol=2e-8)
    np.testing.assert_allclose(tcp[0], tcp[1], atol=1e-15)
    mimic = tree.find('joint[@name="right_finger_joint"]/mimic')
    assert mimic.get('joint') == 'left_finger_joint' and mimic.get('multiplier') == '1'


@pytest.mark.parametrize('robot', ['par6', 'parol6'])
def test_pneumatic_finite_open_close_cycle_is_stable_and_force_limited(robot):
    model = mujoco.MjModel.from_xml_path(str(ROOT/f'mujoco/{robot}/scene_pneumatic.xml'))
    data = mujoco.MjData(model)
    actuator = model.actuator('gripper_actuator').id
    for target in [.0035, 0.]:
        data.ctrl[:6] = 0.; data.ctrl[actuator] = target
        for _ in range(400):
            mujoco.mj_step(model, data)
            assert np.isfinite(data.qpos).all()
            assert abs(data.actuator_force[actuator]) <= 20.+1e-9
        assert abs(data.qpos[6]-target) < 1e-4
        assert abs(data.qpos[7]-data.qpos[6]) < 2e-5
    assert not any(item.number for item in data.warning)
    assert sum(model.body(name).mass[0] for name in PARTS) == pytest.approx(.195, abs=1e-12)
