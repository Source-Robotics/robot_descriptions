"""Build the original pneumatic assembly with movable jaws, using local sources.

The preserved L6 STL contains all three rigid assemblies. Exact source triangle
records are partitioned into housing and two jaws; contacts are cached convex
pieces. Mass and valve/servo parameters remain explicitly unverified estimates.
"""

import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco
import numpy as np
import trimesh

from split_motor_visuals import ROOT, MJCF, URDF, read_parts
from build_gripper_contacts import convex_parts
from build_msg_jaws import stl_bytes
from sync_model_dynamics import numbers, write_native, write_urdf

CONFIG = ROOT/'source/pneumatic/model.json'
PARTS = ('pneumatic_gripper_link', 'left_finger_link', 'right_finger_link')


def configuration():
    return json.loads(CONFIG.read_text(encoding='utf-8'))


def assets(config):
    source = ROOT/config['source_mesh']
    assert hashlib.sha256(source.read_bytes()).hexdigest() == config['source_sha256']
    blob, records, groups = read_parts(source)
    ids = [i for selected in config['components'].values() for i in selected]
    assert sorted(ids) == list(range(len(groups)))
    offset = np.array(config['source_mesh_translation_m'])
    cache = ROOT/'source/pneumatic/contact_cache'
    cache.mkdir(exist_ok=True)
    mesh_data, solids = {}, {}
    for name in PARTS:
        selected = np.sort(np.concatenate([groups[i] for i in config['components'][name]]))
        mesh_data[name] = blob[:80]+len(selected).to_bytes(4, 'little')+records[selected].tobytes()
        vertices = records['vertex'][selected].reshape(-1, 3).astype(float)
        mesh = trimesh.Trimesh(vertices=vertices, faces=np.arange(len(vertices)).reshape(-1, 3), process=True)
        mesh.apply_translation(offset)
        assert mesh.is_volume
        solids[name] = mesh
    volume = sum(mesh.volume for mesh in solids.values())
    density = config['total_mass_kg']/volume
    report = {'source_sha256': config['source_sha256'], 'source_triangle_partition_exact': True,
              'mass_status': config['mass_status'], 'mass_distribution': config['mass_distribution'],
              'total_mass_kg': config['total_mass_kg'], 'parts': {}}
    for name, mesh in solids.items():
        signature = hashlib.sha256(mesh_data[name]+offset.tobytes()+b'convex_parts_6_v1').hexdigest()
        meta = cache/f'{name}.json'
        saved = json.loads(meta.read_text()) if meta.exists() else {}
        count = saved.get('pieces', 0)
        reusable = saved.get('signature') == signature and count > 0 and all((cache/f'{name}_{i:02d}.STL').exists() for i in range(count))
        if not reusable:
            pieces = convex_parts(mesh)
            for i, piece in enumerate(pieces):
                (cache/f'{name}_{i:02d}.STL').write_bytes(stl_bytes(piece, 'Pneumatic convex contact; metres'))
            count = len(pieces)
            meta.write_text(json.dumps({'signature': signature, 'pieces': count}, indent=2)+'\n', encoding='utf-8')
        for package, folder in [(MJCF, 'assets/pneumatic'), (URDF, 'meshes/pneumatic')]:
            dest = package/folder
            dest.mkdir(parents=True, exist_ok=True)
            (dest/f'{name}.STL').write_bytes(mesh_data[name])
            for i in range(count):
                shutil.copyfile(cache/f'{name}_{i:02d}.STL', dest/f'{name}_contact_{i:02d}.STL')
        report['parts'][name] = {'mass_kg': float(mesh.volume*density), 'com_m': mesh.center_mass.tolist(),
            'inertia_kg_m2': (mesh.moment_inertia*density).tolist(), 'collision_pieces': count,
            'visual_triangles': (len(mesh_data[name])-84)//50, 'bounds_m': mesh.bounds.tolist()}
    return report


def build_urdf(config, report):
    tree = ET.parse(URDF/'par6_flange.urdf')
    root = tree.getroot(); root.set('name', 'par6_pneumatic')
    for tag, name in [('link', 'flange_link'), ('joint', 'flange_tool_fixed_joint'), ('joint', 'tcp_fixed_joint')]:
        root.remove(root.find(f'{tag}[@name="{name}"]'))
    for name in PARTS:
        entry = report['parts'][name]
        link = ET.SubElement(root, 'link', name=name)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass'); ET.SubElement(inertial, 'inertia')
        write_urdf(inertial, entry['mass_kg'], entry['com_m'], np.array(entry['inertia_kg_m2']))
        visual = ET.SubElement(link, 'visual', name=name+'_visual')
        ET.SubElement(visual, 'origin', xyz=numbers(config['source_mesh_translation_m']), rpy='0 0 0')
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh', filename=f'meshes/pneumatic/{name}.STL')
        mat = ET.SubElement(visual, 'material', name='pneumatic_body' if name == PARTS[0] else 'pneumatic_jaw')
        ET.SubElement(mat, 'color', rgba=config['colors']['body' if name == PARTS[0] else 'jaw'])
        for i in range(entry['collision_pieces']):
            collision = ET.SubElement(link, 'collision', name=f'contact_{name}_{i:02d}')
            ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='0 0 0')
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=f'meshes/pneumatic/{name}_contact_{i:02d}.STL')
    joint = ET.SubElement(root, 'joint', name='pneumatic_gripper_fixed_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='flange'); ET.SubElement(joint, 'child', link=PARTS[0])
    ET.SubElement(joint, 'origin', xyz=numbers(config['flange_to_body_xyz_m']), rpy=numbers(config['flange_to_body_rpy_rad']))
    for side, axis in [('left', '0 -1 0'), ('right', '0 1 0')]:
        joint = ET.SubElement(root, 'joint', name=f'{side}_finger_joint', type='prismatic')
        ET.SubElement(joint, 'parent', link=PARTS[0]); ET.SubElement(joint, 'child', link=f'{side}_finger_link')
        ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0'); ET.SubElement(joint, 'axis', xyz=axis)
        ET.SubElement(joint, 'limit', lower='0', upper=numbers([config['travel_per_finger_m']]),
                      effort=numbers([config['actuator_force_limit_n']]), velocity=numbers([config['joint_velocity_limit_m_s']]))
        ET.SubElement(joint, 'dynamics', damping=numbers([config['joint_damping_ns_per_m']]), friction='0')
        if side == 'right': ET.SubElement(joint, 'mimic', joint='left_finger_joint', multiplier='1', offset='0')
    tcp = np.array(config['tcp_in_source_mesh_m'])+config['source_mesh_translation_m']
    joint = ET.SubElement(root, 'joint', name='tcp_fixed_joint', type='fixed')
    ET.SubElement(joint, 'parent', link=PARTS[0]); ET.SubElement(joint, 'child', link='tcp')
    ET.SubElement(joint, 'origin', xyz=numbers(tcp), rpy='0 3.141592653589793 0')
    ET.indent(tree, space='  '); tree.write(URDF/'par6_pneumatic.urdf', encoding='utf-8', xml_declaration=True)


def build_native(config, report):
    tree = ET.parse(MJCF/'par6_flange.xml')
    root = tree.getroot(); root.set('model', 'PAR6_PNEUMATIC')
    flange = root.find('.//body[@name="flange"]')
    for child in list(flange): flange.remove(child)
    root.find('asset').remove(root.find('asset/mesh[@name="flange"]'))
    body = ET.SubElement(flange, 'body', name=PARTS[0], pos=numbers(config['flange_to_body_xyz_m']), quat='0 0 1 0')
    for name in PARTS:
        entry = report['parts'][name]
        item = body if name == PARTS[0] else ET.SubElement(body, 'body', name=name)
        inertial = ET.SubElement(item, 'inertial')
        write_native(inertial, entry['mass_kg'], entry['com_m'], entry['inertia_kg_m2'])
        if name != PARTS[0]:
            side = name.split('_')[0]
            ET.SubElement(item, 'joint', name=f'{side}_finger_joint', type='slide', axis='0 -1 0' if side == 'left' else '0 1 0',
                range=numbers([0, config['travel_per_finger_m']]), damping=numbers([config['joint_damping_ns_per_m']]), armature='0.0001')
        ET.SubElement(root.find('asset'), 'mesh', name=name+'_visual', file=f'pneumatic/{name}.STL')
        ET.SubElement(item, 'geom', name=name+'_visual', type='mesh', mesh=name+'_visual',
                      pos=numbers(config['source_mesh_translation_m']), rgba=config['colors']['body' if name == PARTS[0] else 'jaw'],
                      contype='0', conaffinity='0', mass='0', group='2')
        for i in range(entry['collision_pieces']):
            mesh = f'contact_{name}_{i:02d}'
            ET.SubElement(root.find('asset'), 'mesh', name=mesh, file=f'pneumatic/{name}_contact_{i:02d}.STL')
            ET.SubElement(item, 'geom', name=mesh, type='mesh', mesh=mesh, rgba='0 0 0 0', group='3', mass='0',
                          friction='1 0.005 0.0001', solref='0.005 1', solimp='0.9 0.95 0.001')
    tcp = np.array(config['tcp_in_source_mesh_m'])+config['source_mesh_translation_m']
    ET.SubElement(body, 'site', name='tcp', pos=numbers(tcp), quat='0 0 1 0', size='0.005', rgba='1 0 0 1')
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'joint', joint1='right_finger_joint', joint2='left_finger_joint', polycoef='0 1 0 0 0', solref='0.005 1')
    ET.SubElement(root.find('actuator'), 'position', name='gripper_actuator', joint='left_finger_joint',
                  kp=numbers([config['position_gain_n_per_m']]), ctrllimited='true', ctrlrange=numbers([0, config['travel_per_finger_m']]),
                  forcelimited='true', forcerange=numbers([-config['actuator_force_limit_n'], config['actuator_force_limit_n']]))
    for key in root.findall('keyframe/key'):
        key.set('qpos', key.get('qpos')+' 0 0'); key.set('ctrl', key.get('ctrl')+' 0')
    ET.indent(tree, space='  '); tree.write(MJCF/'par6_pneumatic.xml', encoding='utf-8', xml_declaration=True)
    (MJCF/'scene_pneumatic.xml').write_text((MJCF/'scene_flange.xml').read_text().replace('flange', 'pneumatic'), encoding='utf-8')
    mujoco.MjModel.from_xml_path(str(MJCF/'scene_pneumatic.xml'))


def metadata():
    for package, native in [(MJCF, True), (URDF, False)]:
        path = package/'sim2bot.robot.json'
        data = json.loads(path.read_text())
        data['variants'] = [v for v in data['variants'] if v['id'] != 'pneumatic']
        entry = {'id': 'pneumatic', 'label': 'Pneumatic gripper',
                 'entry': 'scene_pneumatic.xml' if native else 'par6_pneumatic.urdf',
                 'endEffectorBodies': [PARTS[0]], 'gripperActuators': ['gripper_actuator'] if native else []}
        if (package/'par6_pneumatic.png').exists(): entry['preview'] = 'par6_pneumatic.png'
        data['variants'].append(entry)
        path.write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')


def documentation(report):
    rows = '\n'.join(f'| {name} | {entry["mass_kg"]*1000:.3f} | {entry["collision_pieces"]} |'
                     for name, entry in report['parts'].items())
    text = f'''# Pneumatic gripper

The original PAROL6 L6 assembly is available as `pneumatic` on both arms.
Its preserved source triangles are partitioned into the housing/air fittings
and two moving jaw assemblies. Black jaws and a silver housing are visuals;
convex contact pieces follow the actual selected geometry.

## Load and move

Open `scene_pneumatic.xml` in either MuJoCo package, for example:

```bash
python -m mujoco.viewer --mjcf scene_pneumatic.xml
```

URDF entries are `par6_pneumatic.urdf` and `parol6_pneumatic.urdf`.
The ROS display launch accepts `end_effector:=pneumatic`.

`left_finger_joint` and `right_finger_joint` are opposing prismatic joints.
Both use displacement 0 for open and +0.0035 m for closed; the right joint
mimics the left in URDF and is coupled to it by a native equality constraint.
Set MuJoCo `gripper_actuator` control to 0 or 0.0035 m for the endpoints.
The CAD inner clearance is approximately 7.1 mm open and 0.1 mm closed.

The force-limited position actuator approximates valve-command endpoints.
It does not model compressed air, pressure-dependent force, or valve dynamics.
Kp = 8000 N/m, actuator force limit = +/-20 N, and finger damping = 2 Ns/m
are simulation estimates, not verified hardware ratings. URDF controller gains
belong in the consuming simulator/controller; its joint limits are included.

## Mount and TCP

The assembly rear face is source Z = -36 mm. Mesh origins translate it
+36 mm to body Z = 0, then the body mounts 15 mm behind the common flange
reference with a 180-degree Y rotation. This keeps the same approximately
1 mm housing clearance as the other shared tools. The flange offset is
57.1 mm on PAR6 and 50.5 mm on PAROL6; no raw wrist-specific offset is baked
into the pneumatic meshes.

The operational TCP follows the legacy MHZ2-10D reference point in the source
assembly: XYZ (+45.25, 0, -62.8) mm. Rebasing/mounting carries that point with
the tool. J6-to-TCP axial reach is 68.9 mm on PAR6 and 62.3 mm on PAROL6;
the lateral offset is 45.25 mm. Its orientation matches the other package
TCPs. This is a retained operational reference, not an independently measured
grasp center. Source geometry includes a small jaw asymmetry.

## Estimated physical data

The legacy RCB MHZ2-10D configuration lists a **195 g total marked TODO**.
No pneumatic mass measurement was supplied. That total is distributed by
solid CAD volume; each rigid part's COM and full central inertia are computed
from its CAD volume with the resulting uniform density. These are explicitly
unverified simulation estimates, separate from measured MSG/SSG48 masses.

| Rigid assembly | Estimated mass (g) | Convex contact pieces |
| --- | ---: | ---: |
{rows}

The authoring inputs are `source/pneumatic/model.json` and the byte-preserved
`source/parol6/original/meshes/L6.STL`. Normal builds need no sibling API or RCB
checkout. See `kinematics_review/pneumatic.json` for FK checks and transforms.
'''
    for package in [MJCF, URDF]:
        (package/'PNEUMATIC.md').write_text(text, encoding='utf-8')


def build():
    config = configuration()
    report = assets(config)
    build_urdf(config, report); build_native(config, report); metadata(); documentation(report)
    (ROOT/'reports/pneumatic_geometry.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print('Built pneumatic gripper with two moving jaws; legacy 195 g total and finite servo settings are estimates.', flush=True)
    return report


if __name__ == '__main__': build()
