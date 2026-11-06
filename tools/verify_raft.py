# SPDX-License-Identifier: Apache-2.0
"""Check RAFT's packages against RAFT's model reference (source/raft/reference.json) and each other.

The reference carries sample poses with RAFT's own forward kinematics and gravity torques, so both formats are
compared with the numbers RAFT's firmware uses, not with this repository's writer. Also checks the masses, the
ranges against the measured stops, the keyframes, the palette colours, the visual pieces and the mesh paths.
"""
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))   # isolated (-I) mode, or called from elsewhere
from build_raft import MUJOCO, SOURCE, URDF, materials, partitions, piece_key, reference
from model_colors import load as load_palette

JOINTS = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'trigger_joint']
TOL = 1e-9


def stl_triangles(path):
    data = Path(path).read_bytes()
    count = int.from_bytes(data[80:84], 'little')
    assert len(data) == 84+50*count, f'{path}: not a binary STL'
    return count


def pose(m, d, kind, name):
    T = np.eye(4)
    if kind == 'site':
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, name); T[:3, :3] = d.site_xmat[i].reshape(3, 3); T[:3, 3] = d.site_xpos[i]
    else:
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name); T[:3, :3] = d.xmat[i].reshape(3, 3); T[:3, 3] = d.xpos[i]
    return T


def indices(m):
    ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in JOINTS]
    assert min(ids) >= 0, 'joint names'
    return [m.jnt_qposadr[i] for i in ids], [m.jnt_dofadr[i] for i in ids], np.array([m.jnt_range[i] for i in ids])


def set_q(m, d, adr, q):
    d.qpos[:] = 0; d.qvel[:] = 0
    for k, a in enumerate(adr):
        d.qpos[a] = q[k]
    mujoco.mj_forward(m, d)


def load_urdf(path):
    """MuJoCo reads the URDF directly; it needs the mesh subfolders kept (not part of the shipped file)"""
    text = Path(path).read_text(encoding='utf-8').replace(
        '<robot name="raft">', '<robot name="raft"><mujoco><compiler strippath="false" fusestatic="false" discardvisual="true"/></mujoco>', 1)
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', dir=Path(path).parent, delete=False, encoding='utf-8') as f:
        f.write(text)
    try:
        return mujoco.MjModel.from_xml_path(f.name)
    finally:
        Path(f.name).unlink()


def check_model(scene, urdf, ref):
    """both formats against the reference; returns counts for the report"""
    m = mujoco.MjModel.from_xml_path(str(scene)); d = mujoco.MjData(m)
    adr, dof, lim = indices(m)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(1, m.nbody)]
    assert names == [link['name'] for link in ref['links']], names
    acts = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in range(m.nu)]
    assert acts == [j.replace('_joint', '_actuator') for j in JOINTS], acts
    assert np.allclose(lim, [j['range'] for j in ref['joints']], atol=1e-12), 'ranges'
    worst = dict(tcp=0., trigger=0., gravity=0.)
    for sample in ref['checks']['samples']:
        set_q(m, d, adr, sample['q'])
        worst['tcp'] = max(worst['tcp'], np.abs(pose(m, d, 'site', 'tcp')-sample['tcp']).max())
        worst['trigger'] = max(worst['trigger'], np.abs(pose(m, d, 'body', 'trigger_link')-sample['trigger_link']).max())
        worst['gravity'] = max(worst['gravity'], np.abs(d.qfrc_bias[dof]-sample['gravity_torque']).max())
    assert max(worst.values()) < TOL, f'MuJoCo package against the RAFT reference: {worst}'
    set_q(m, d, adr, np.zeros(7))
    assert np.allclose(d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'tcp')], ref['checks']['tcp_home'], atol=1e-12)
    assert abs(m.body_mass.sum()-ref['checks']['mass_total']) < 1e-12, 'total mass'
    assert abs(m.body_mass[2:].sum()-ref['checks']['mass_moving']) < 1e-12, 'moving mass'
    # keyframes: named as in the reference, inside the ranges, free of contacts, held by the motors
    keys = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_KEY, k) for k in range(m.nkey)]
    assert keys == [k['name'] for k in ref['keyframes']], keys
    drift = 0.
    for k in range(m.nkey):
        q = m.key_qpos[k][adr]
        assert np.all(q >= lim[:, 0]-1e-12) and np.all(q <= lim[:, 1]+1e-12), f'keyframe {keys[k]} outside the ranges'
        mujoco.mj_resetDataKeyframe(m, d, k); mujoco.mj_forward(m, d)
        assert d.ncon == 0, f'keyframe {keys[k]}: {d.ncon} contacts'
        q0 = d.qpos.copy()
        for _ in range(1000):
            mujoco.mj_forward(m, d)
            d.ctrl[:] = np.clip(d.qfrc_bias[dof], *m.actuator_ctrlrange.T)
            mujoco.mj_step(m, d)
        drift = max(drift, np.abs(d.qpos-q0).max())
    assert drift < np.deg2rad(.1), f'gravity compensation drifts {np.rad2deg(drift):.3f} deg'
    # the URDF, read by MuJoCo, against the MuJoCo package
    u = load_urdf(urdf); du = mujoco.MjData(u)
    adr_u, dof_u, lim_u = indices(u)
    assert np.allclose(lim_u, lim, atol=1e-12), 'URDF ranges'
    worst_u = 0.
    for sample in ref['checks']['samples']:
        set_q(u, du, adr_u, sample['q']); set_q(m, d, adr, sample['q'])
        worst_u = max(worst_u, np.abs(pose(u, du, 'body', 'tcp')-pose(m, d, 'site', 'tcp')).max(),
                      np.abs(du.qfrc_bias[dof_u]-d.qfrc_bias[dof]).max())
    assert worst_u < TOL, f'URDF against the MuJoCo package: {worst_u}'
    return {'samples': len(ref['checks']['samples']), 'keyframes': m.nkey, 'max_error': max(max(worst.values()), worst_u)}


def check_appearance(ref, parts, palette):
    mats = materials(palette)
    m = mujoco.MjModel.from_xml_path(str(MUJOCO/'raft.xml'))
    for name, (rgba, spec, shine) in mats.items():
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_MATERIAL, name)
        assert i >= 0 and np.allclose(m.mat_rgba[i], rgba, atol=1e-9), name
    visual = 0
    for g in range(m.ngeom):
        if m.geom_group[g] != 2:
            continue
        visual += 1
        link = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
        piece = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g])[len(link)+1:]
        assert mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MATERIAL, m.geom_matid[g]) == f'raft_{piece_key(link, piece)}', (link, piece)
    pieces = sum(len(p['parts']) for p in parts.values())
    assert visual == pieces, (visual, pieces)
    root = ET.parse(URDF/'raft.urdf').getroot()
    seen = 0
    for link in root.findall('link'):
        for v in link.findall('visual'):
            seen += 1
            material = v.find('material')
            piece = Path(v.find('geometry/mesh').get('filename')).stem[len(link.get('name'))+1:]
            assert material.get('name') == f'raft_{piece_key(link.get("name"), piece)}', piece
            assert np.allclose([float(x) for x in material.find('color').get('rgba').split()], mats[material.get('name')][0], atol=1e-9)
    assert seen == pieces, (seen, pieces)
    return pieces


def check_pieces(ref, parts):
    for link in ref['links']:
        name, record = link['name'], parts[link['name']]
        full = stl_triangles(SOURCE/record['source'])
        assert full == record['triangles'] == sum(stl_triangles(SOURCE/p['file']) for p in record['parts'].values()), name
        for package, meshdir in [(MUJOCO, 'assets'), (URDF, 'meshes')]:
            assert (package/meshdir/f'{name}.stl').read_bytes() == (SOURCE/record['source']).read_bytes(), (package, name)
            for p in record['parts'].values():
                assert (package/meshdir/p['file']).read_bytes() == (SOURCE/p['file']).read_bytes(), (package, p['file'])
    return sum(p['triangles'] for p in parts.values())


def check_package(package, native):
    """an extracted ZIP: entry files load, assets stay inside, the index fits the model"""
    package = Path(package).resolve()
    data = json.loads((package/'sim2bot.robot.json').read_text(encoding='utf-8'))
    assert [v['id'] for v in data['variants']] == ['leader'], data['variants']
    entry = package/data['entry']
    if native:
        m = mujoco.MjModel.from_xml_path(str(entry))
        assert m.nq == 7 and m.nu == 7
        for name in data['armActuators']+data['gripperActuators']:
            assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name) >= 0, name
        for name in data['tcpSites']:
            assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, name) >= 0, name
        for name in data['endEffectorBodies']:
            assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name) >= 0, name
        counts = {'bodies': m.nbody-1, 'geoms': m.ngeom-1}
    else:
        root = ET.parse(entry).getroot()
        for mesh in root.iter('mesh'):
            path = (entry.parent/mesh.get('filename')).resolve()
            assert path.is_relative_to(package) and path.is_file(), mesh.get('filename')
        joints = [j.get('name') for j in root.findall('joint')]
        for name in data['armActuators']+data['gripperActuators']:
            assert name in joints, name
        m = load_urdf(entry)
        counts = {'links': len(root.findall('link')), 'meshes': len(list(root.iter('mesh')))}
    return {'variants_checked': 1, **counts, 'crc_and_assets_valid': True}


def verify():
    ref, parts, palette = reference(), partitions(), load_palette()
    triangles = check_pieces(ref, parts)
    pieces = check_appearance(ref, parts, palette)
    result = check_model(MUJOCO/'scene.xml', URDF/'raft.urdf', ref)
    stops = [j['measured_stop_deg'] for j in ref['joints']]
    for joint, stop in zip(ref['joints'], stops):
        end = -stop if joint['name'] == 'trigger_joint' else stop       # the trigger opens toward negative angles
        assert np.isclose(end, joint['range_deg']).any(), (joint['name'], stop)
    for package, urdf in [(MUJOCO, False), (URDF, True)]:
        check_package(package, not urdf)
    print(f'RAFT: {pieces} coloured pieces ({triangles} triangles), {result["samples"]} reference poses, '
          f'{result["keyframes"]} keyframes; both formats agree with RAFT\'s reference to {result["max_error"]:.1e}.', flush=True)
    return result


if __name__ == '__main__':
    verify()
