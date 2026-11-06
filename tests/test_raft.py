import json
import xml.etree.ElementTree as ET

import build_raft
from model_colors import load
from model_reference import ROOT
from ros_support.robot_description import select_model
import verify_raft


def test_raft_both_formats_agree_with_the_raft_reference_kinematics_and_gravity():
    result = verify_raft.check_model(ROOT/'mujoco/raft/scene.xml', ROOT/'urdf/raft_description/raft.urdf',
                                     build_raft.reference())
    assert result['samples'] == 40 and result['max_error'] < 1e-9


def test_raft_visual_pieces_keep_every_cad_triangle_and_follow_the_shared_palette():
    ref, parts = build_raft.reference(), build_raft.partitions()
    assert verify_raft.check_pieces(ref, parts) == sum(p['triangles'] for p in parts.values())
    assert verify_raft.check_appearance(ref, parts, load()) == 26


def test_raft_committed_models_are_what_the_build_writes(tmp_path, monkeypatch):
    monkeypatch.setattr(build_raft, 'MUJOCO', tmp_path/'mujoco/raft')
    monkeypatch.setattr(build_raft, 'URDF', tmp_path/'urdf/raft_description')
    build_raft.build()
    for package, name in [('mujoco/raft', 'raft.xml'), ('mujoco/raft', 'scene.xml'),
                          ('urdf/raft_description', 'raft.urdf'), ('urdf/raft_description', 'config/joint_names.yaml')]:
        built = (tmp_path/package/name).read_text(encoding='utf-8').replace('\r\n', '\n')
        assert built == (ROOT/package/name).read_text(encoding='utf-8').replace('\r\n', '\n'), name


def test_raft_ranges_end_at_the_measured_stops_and_keyframes_stay_inside():
    ref = build_raft.reference()
    for joint in ref['joints']:
        stop = joint['measured_stop_deg']
        assert any(abs(end-(-stop if joint['name'] == 'trigger_joint' else stop)) < 1e-9 for end in joint['range_deg'])
        assert joint['range'][0] < joint['range'][1]
    for key in ref['keyframes']:
        assert all(j['range'][0]-1e-12 <= q <= j['range'][1]+1e-12 for j, q in zip(ref['joints'], key['qpos'])), key['name']


def test_raft_ros_selection_has_one_configuration_and_index_matches_the_model():
    assert select_model('raft') == 'raft.urdf' == select_model('raft', 'msg', 'default')
    data = json.loads((ROOT/'urdf/raft_description/sim2bot.robot.json').read_text())
    joints = [j.get('name') for j in ET.parse(ROOT/'urdf/raft_description/raft.urdf').findall('joint')]
    assert all(name in joints for name in data['armActuators']+data['gripperActuators'])
    assert data['packageVersion'] == ET.parse(ROOT/'urdf/par6_description/package.xml').findtext('version')
