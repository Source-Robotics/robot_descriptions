"""Build PAROL6 from included CAD exports and the generated PAR6 tool profiles.

Optional one-time source capture: --import-source /path/to/exported/parol6.
Normal builds read only files in this description repository.
"""

from pathlib import Path
import argparse
import copy
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco
import numpy as np

from parol6_model import (ROOT, SOURCE, ORIGINAL, URDF, MJCF, PROFILES,
                          ARM_JOINTS, ARM_LINKS, PAR6_FROM_PAROL6,
                          SOURCE_FROM_PAROL6, FOREARM_DISTANCE,
                          FLANGE_DISTANCE, LOWER_ARM_MESH_OFFSET,
                          config, normalized_joint_limits, wrist_mount_geometry)
from split_motor_visuals import read_parts, BLACK
from sync_model_dynamics import numbers

PAR6_URDF = ROOT / 'urdf/par6_description'
PAR6_MJCF = ROOT / 'mujoco/par6'
GRAY = '0.79216 0.81961 0.93333 1'
LABELS = dict(zip(PROFILES, ('Flange only', 'MSG - Solid jaws', 'MSG - Soft TPU jaws',
                           'SSG48 - Solid jaws', 'SSG48 - Fin-ray jaws', 'Pneumatic gripper')))
SPLITS = {
    'L1': {'shoulder_structure_visual': [4, 16, 17, 28, 31, 32],
           'j1_actuator_visual': [0, 1, 5, 7, 8, 9, 10, 18, 20, 21, 22, 23, 29],
           'j2_actuator_visual': [2, 3, 6, 11, 12, 13, 14, 15, 19, 24, 25, 26, 27, 30]},
    'L2': {'upper_arm_structure_visual': [2, 3, 10, 16],
           'j3_actuator_visual': [0, 1, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 17]},
}


from package_layout import documentation, previews

def write_xml(tree, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)


def import_source(directory):
    directory = Path(directory).resolve()
    names = ['urdf/parol6.urdf', 'package.xml'] + [f'meshes/{name}.STL' for name in ['base_link', *[f'L{i}' for i in range(1, 7)]]]
    for name in names:
        if not (directory / name).is_file():
            raise ValueError(f'Missing supplied PAROL6 input: {name}')
    for name in names:
        target = ORIGINAL / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(directory / name, target)
    data = {
        'schema_version': 1,
        'source': 'User-supplied SolidWorks PAROL6 URDF export',
        'original_sha256': {name: hashlib.sha256((ORIGINAL/name).read_bytes()).hexdigest() for name in names},
        'joint_policy': config()['joint_policy'],
        'geometry': config()['geometry'],
        'par6_from_parol6_joint_signs': PAR6_FROM_PAROL6.tolist(),
        'source_from_parol6_joint_signs': SOURCE_FROM_PAROL6.tolist(),
        'dynamics_policy': 'Inherit PAR6 mass, local COM and central inertia in corresponding CAD link frames, separately for each format. These are user-authorized shared estimates for PAROL6, not a PAROL6 measurement.',
        'actuation_policy': 'User selected finite position control with torque limits. Retain PAR6 simulation estimates; open-loop hardware does not imply infinite stiffness.',
        'tool_policy': config()['tool_policy'],
        'arm_meshes': dict(zip(['base_link', *ARM_LINKS[:5]], ['base_link', 'L1', 'L2', 'L3', 'L4', 'L5'])),
        'visual_components': SPLITS,
    }
    (SOURCE/'model.json').write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8', newline='\n')


def visual_parts(settings):
    result = {}
    for source, definitions in settings['visual_components'].items():
        blob, records, groups = read_parts(ORIGINAL/f'meshes/{source}.STL')
        assigned = [i for ids in definitions.values() for i in ids]
        if sorted(assigned) != list(range(len(groups))):
            raise ValueError(f'{source}: components must be a complete disjoint partition')
        for part, ids in definitions.items():
            triangles = np.sort(np.concatenate([groups[i] for i in ids]))
            data = blob[:80] + len(triangles).to_bytes(4, 'little') + records[triangles].tobytes()
            for directory in [URDF/'meshes/arm/visual', MJCF/'assets/visual']:
                directory.mkdir(parents=True, exist_ok=True)
                (directory/f'{part}.STL').write_bytes(data)
            result[part] = {'source': f'meshes/{source}.STL', 'components': ids, 'triangles': len(triangles)}
    return result


def replace_urdf_arm(root, source, settings):
    for index, joint_name in enumerate(ARM_JOINTS, start=1):
        joint = root.find(f'joint[@name="{joint_name}"]')
        imported = source.find(f'joint[@name="L{index}"]')
        joint.set('type', imported.get('type'))
        # PAR6 axes/orientations are already present in this template.
        if index == 5:
            joint.find('origin').set('xyz', numbers([0., 0., -FOREARM_DISTANCE]))
        for child in joint.findall('limit'):
            joint.remove(child)
        limit = copy.deepcopy(imported.find('limit'))
        low, high = normalized_joint_limits(source, index)
        limit.set('lower', numbers([low]));limit.set('upper', numbers([high]))
        joint.append(limit)
    root.find('joint[@name="flange_fixed_joint"]/origin').set('xyz', numbers([0., 0., FLANGE_DISTANCE]))
    for link_name, mesh_name in settings['arm_meshes'].items():
        link = root.find(f'link[@name="{link_name}"]')
        for element in link.findall('visual') + link.findall('collision'):
            link.remove(element)
        parts = settings['visual_components'].get(mesh_name, {mesh_name: None})
        for part in parts:
            black = '_actuator_' in part
            visual = ET.SubElement(link, 'visual', name=part)
            offset = LOWER_ARM_MESH_OFFSET if link_name == 'wrist_1_link' else np.zeros(3)
            ET.SubElement(visual, 'origin', xyz=numbers(offset), rpy='0 0 0')
            geometry = ET.SubElement(visual, 'geometry')
            relative = f'meshes/arm/visual/{part}.STL' if mesh_name in settings['visual_components'] else f'meshes/arm/{mesh_name}.STL'
            ET.SubElement(geometry, 'mesh', filename=relative)
            material = ET.SubElement(visual, 'material', name=part[:2]+'_actuator_black' if black else 'parol6_structure')
            ET.SubElement(material, 'color', rgba=BLACK if black else GRAY)
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', xyz=numbers(offset), rpy='0 0 0')
        geometry = ET.SubElement(collision, 'geometry')
        ET.SubElement(geometry, 'mesh', filename=f'meshes/arm/{mesh_name}.STL')


def copy_urdf_assets(root, settings):
    for element in root.findall('.//mesh'):
        relative = Path(element.get('filename'))
        target = URDF/relative
        if relative.parts[:2] == ('meshes', 'arm'):
            if relative.parts[2] != 'visual':
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ORIGINAL/'meshes'/relative.name, target)
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(PAR6_URDF/relative, target)


def replace_native_arm(root, source, settings, profile):
    for index, (joint_name, body_name) in enumerate(zip(ARM_JOINTS, ARM_LINKS), start=1):
        body = root.find(f'.//body[@name="{body_name}"]')
        imported = source.find(f'joint[@name="L{index}"]')
        # Public arm frames match PAR6; the source's rounded forearm is corrected.
        if index == 5:
            body.set('pos', numbers([0., 0., -FOREARM_DISTANCE]))
        joint = body.find(f'joint[@name="{joint_name}"]')
        joint.set('limited', 'false' if imported.get('type') == 'continuous' else 'true')
        if imported.get('type') != 'continuous':
            bounds = numbers(normalized_joint_limits(source, index))
            joint.set('range', bounds)
            actuator = root.find(f'actuator/position[@joint="{joint_name}"]')
            actuator.set('ctrlrange', bounds)
    root.find('.//body[@name="flange"]').set('pos', numbers([0., 0., FLANGE_DISTANCE]))
    for geom in root.findall('.//body[@name="wrist_1_link"]/geom'):
        geom.set('pos', numbers(LOWER_ARM_MESH_OFFSET))
    # Preserve the PAR6 native fixed-base estimate rather than re-inferring
    # its mass from the different PAROL6 base mesh volume.
    template = mujoco.MjModel.from_xml_path(str(PAR6_MJCF/f'par6_{profile}.xml'))
    base_id = template.body('base_link').id
    base = root.find('.//body[@name="base_link"]')
    inertial = base.find('inertial')
    if inertial is None:
        inertial = ET.SubElement(base, 'inertial')
    inertial.attrib.update(mass=numbers([template.body_mass[base_id]]), pos=numbers(template.body_ipos[base_id]),
                           quat=numbers(template.body_iquat[base_id]), diaginertia=numbers(template.body_inertia[base_id]))
    asset = root.find('asset')
    for link_name, mesh_name in settings['arm_meshes'].items():
        mesh = asset.find(f'mesh[@name="{link_name}"]')
        mesh.set('file', f'arm/{mesh_name}.STL')
        if mesh_name not in settings['visual_components']:
            continue
        for part in settings['visual_components'][mesh_name]:
            asset.find(f'mesh[@name="{part}"]').set('file', f'visual/{part}.STL')
        body = root.find(f'.//body[@name="{link_name}"]')
        for geom in body.findall('geom'):
            if geom.get('name', '').endswith('structure_visual'):
                geom.attrib.pop('material', None)
                geom.set('rgba', GRAY)
    for tag, name in [('texture', 'par6_badge_texture'), ('material', 'par6_badge_material')]:
        entry = asset.find(f'{tag}[@name="{name}"]')
        if entry is not None:
            asset.remove(entry)
    # PAR6 keyframes now use the public PAROL6 coordinates directly.


def copy_native_assets(root, settings):
    for mesh in root.findall('asset/mesh'):
        relative = Path(mesh.get('file'))
        target = MJCF/'assets'/relative
        if relative.parts[0] == 'visual' and relative.stem in {p for parts in settings['visual_components'].values() for p in parts}:
            continue  # Exact partitions already written above.
        source = ORIGINAL/'meshes'/relative.name if relative.parts[0] == 'arm' else PAR6_MJCF/'assets'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)


def metadata_and_support():
    version = ET.parse(PAR6_URDF/'package.xml').findtext('version')
    for template, target in [(PAR6_URDF, URDF), (PAR6_MJCF, MJCF)]:
        data = json.loads((template/'sim2bot.robot.json').read_text())
        data['name'] = data['name'].replace('PAR6', 'PAROL6')
        data['sourceUrl'] = 'https://github.com/Source-Robotics/robot_descriptions'
        data['sourceRevision'] = 'PAROL6 included export; see source/parol6/model.json in the authoring repository'
        data['packageVersion'] = version
        data['entry'] = data['entry'].replace('par6_', 'parol6_')
        data.pop('preview', None)
        for variant in data['variants']:
            variant['entry'] = variant['entry'].replace('par6_', 'parol6_')
            variant.pop('preview', None)
            profile = variant['entry'].removeprefix('scene_').removeprefix('parol6_').removesuffix('.xml').removesuffix('.urdf')
            if (previews(target)/f'parol6_{profile}.png').is_file():
                variant['preview'] = f'previews/parol6_{profile}.png'
        if (previews(target)/'parol6_msg.png').is_file():
            data['preview'] = 'previews/parol6_msg.png'
        (target/'sim2bot.robot.json').write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
        shutil.copyfile(ROOT/'LICENSE', target/'LICENSE')
        for document in ['GRASP_FRAMES.md', 'CONTACT_PERFORMANCE.md']:
            text = (documentation(template)/document).read_text(encoding='utf-8').replace('PAR6', 'PAROL6').replace('par6_', 'parol6_')
            (documentation(target)/document).write_text(text, encoding='utf-8')
        (documentation(target)/'CHANGELOG.md').write_text(f'''# PAROL6 description changelog

## {version} - 2026-11-01

- Moved detailed notes and review sheets into docs/, and catalog images into previews/.
- Retained runtime model, scene, mesh and ROS launch entry paths and all physical data.

## 2.13.0 - 2026-10-31

- Corrected SSG48 URDF collision proxies to omit internal transmission racks while preserving their visuals and all inertial properties.
- Corrected short-jaw grasp fixture clearance and added full-stroke internal contact and close/lift/hold/release regression checks.

## 2.12.0 - 2026-10-31

- Added the original pneumatic gripper with independently moving, coupled jaws, collision pieces, finite actuation and estimated CAD mass distribution.
- Included the sixth profile in packages, previews, DH tables and review sheets.

## 2.11.0 - 2026-10-31

- Corrected a5 to 177 mm, moving the wrist origin and lower-arm visual/collision placement outward by 0.65 mm.
- Matched PAR6 housing-to-tool spacing: PAROL6 flange reach is 50.5 mm, moving every tool and TCP inward by 6.6 mm relative to J6.
- Kept public axes, origin orientations, flange-relative tool subtrees, local inertials and finite actuator settings.
- Regenerated all review diagrams, DH tables and model packages.

## 2.10.0 - 2026-10-31

- Added dimensioned orthographic snapshots and tool-specific DH review tables.

## 2.9.1 - 2026-10-31

- Standardized axes/origin orientations to PAR6; retained the historical 176.35 mm forearm and initially shared the 57.1 mm flange reach, superseded above.

## 2.9.0 - 2026-10-31

- First PAROL6 package with shared tools and physical properties, using the supplied export angle convention.
''', encoding='utf-8')
        (documentation(target)/'MIGRATION.md').write_text('''# Importing the supplied PAROL6 export

The original URDF names joints and moving links L1 through L6. They are
renamed in order to base_joint/shoulder_link,
shoulder_joint/upper_arm_link, elbow_joint/forearm_link,
wrist_1_joint/wrist_1_link, wrist_2_joint/wrist_2_link and
wrist_3_joint/wrist_3_link. The old fixed world-to-base joint is omitted;
base_link is the fixed package root. Public axes and origin orientations now
match PAR6, including J6. The forearm is corrected to 177 mm; the lower-arm
mesh origin moves -0.65 mm on local Z and J5 moves to -177 mm. The wrist's
flange reach is 50.5 mm, matching PAR6 spacing against the shorter housing.
Source joint types are retained and bounds converted.
New PAR6 and PAROL6 package angles are identical. When migrating trajectories
from the old 2.9.0 raw-export convention, negate joints 3 through 6 once.
The original source URDF remains unchanged for reference.

The original L6 end-tool geometry is replaced by the selected shared tool.
The physical flange adapter and every flange-relative tool TCP are explicit.
Relative to 2.10.0, recompute FK/IK targets using the corrected arm and flange
transforms: the tool is 6.6 mm closer to J6 and the old 176.35 mm forearm is
now 177 mm. This is a model geometry correction, not a joint-angle sign change.
Imported source masses/tensors are superseded by the requested PAR6 values.
ROS launch files convert portable relative mesh paths to package:// URIs
in memory; the portable URDF files on disk are not rewritten.
''', encoding='utf-8')
        suffix = 'urdf' if target == URDF else 'xml'
        entries = '\n'.join(f'| {LABELS[p]} | `parol6_{p}.{suffix}` |' for p in PROFILES)
        viewing = ('Run `roslaunch parol6_description display.launch end_effector:=msg` in a ROS 1 workspace, or load an individual URDF in your viewer.'
                   if target == URDF else 'Run `python -m mujoco.viewer --mjcf scene_msg.xml` from this package folder. `scene.xml` selects MSG Solid jaws by default.')
        (target/'README.md').write_text(f'''# Source Robotics PAROL6

Self-contained {'URDF' if target == URDF else 'MuJoCo'} description with the supplied PAROL6 arm geometry and the
standardized PAR6 joint convention, plus shared PAR6 end effectors.
No other checkout is needed.

![PAROL6 with MSG Solid jaws](previews/parol6_msg.png)

Detailed notes and dimensioned drawings are in [docs/](docs/); model images are
in [previews/](previews/). Entry files, assets and ROS launch paths are unchanged.

| Tool | Model |
| --- | --- |
{entries}

MSG defaults to Solid jaws; SSG48 defaults to Solid jaws. The alternative
options are MSG Soft TPU jaws and SSG48 Fin-ray jaws.
The original side-gripping pneumatic assembly is also selectable; its jaws
move symmetrically through 3.5 mm each. See [PNEUMATIC.md](docs/PNEUMATIC.md) for
the mounting reference, TCP and explicitly estimated dynamics/actuation.

{viewing}

Read [DH.md](docs/DH.md) and [the PAR6 comparison](docs/KINEMATICS_COMPARISON.md) before
exchanging joint angles with older exports. Public angles, axes and J6
orientation now match PAR6, as does the corrected 177 mm forearm (d4 = -177 mm).
Flange-ending d6 differs: -50.5 mm on PAROL6 versus -57.1 mm on PAR6.
The lower-arm visual/collision origins shift outward 0.65 mm from the old
export, and every shared tool moves 6.6 mm toward J6 to match housing spacing.
The exported L6 tool is replaced by the selected tool.
Black actuator meshes are visual-only partitions; full original arm meshes
remain the collision assets. PAR6's badge artwork is not placed on PAROL6.

Mass, local COM and central inertia are inherited from the corresponding
PAR6 package as requested, not independently measured for PAROL6. Flange mass
is 55.5 g; complete MSG is 380 g and complete SSG48 is 440 g, with 39.3 g per
moving finger/jaw/slider assembly. See [DYNAMICS.md](docs/DYNAMICS.md) for limits of
this assumption and [DYNAMICS_AUDIT.md](docs/DYNAMICS_AUDIT.md) for validation.
See [ACTUATION.md](docs/ACTUATION.md) for finite position-control settings;
open-loop hardware does not imply infinite simulated joint stiffness.
See [KINEMATIC_REVIEW.md](docs/KINEMATIC_REVIEW.md) for dimensioned model snapshots
and tool-specific DH tables for all six configurations.

The authoring repository is https://github.com/Source-Robotics/robot_descriptions.
''', encoding='utf-8')
        (documentation(target)/'DYNAMICS.md').write_text('''# PAROL6 dynamics inheritance

The user specified that PAROL6 has the same mass properties as PAR6. Each
format therefore inherits PAR6's mass, local COM and full central inertia
in corresponding CAD link frames. Those L1-L5 CAD frames have the same local
part convention; the changed joint axes affect motion, not the meaning of a
local mass property. Tool inertials stay in their shared tool frames after the
explicit L6-to-flange adapter. Native drive/armature estimates are inherited.

The old masses/tensors in the supplied SolidWorks URDF are retained only in
the preserved authoring source. They do not override the preferred PAR6 data.
The native fixed-base estimate is copied explicitly, preventing the new base
mesh volume from silently changing it. Its scope remains unverified.

Shared physical tool totals: flange 0.0555 kg; MSG 0.380 kg; SSG48 0.440 kg.
Each MSG/SSG48 moving finger/jaw/slider is 0.0393 kg. Tool COM/tensors and motor rotor
inertias remain the existing unverified estimates; equal jaw masses do not
establish equal COM or inertia for different shapes.

The pneumatic assembly uses an unverified legacy 0.195 kg total, distributed
by CAD volume with COM/central tensors computed from those solids. It is not
included in the user-measured MSG/SSG48 mass claims. See PNEUMATIC.md.

Public PAROL6 and PAR6 joint axes/origin orientations now match, including
J6. Both forearms are corrected to 177 mm. Their generalized dynamics can
still differ because PAROL6's shorter wrist places each tool 6.6 mm closer
to J6, changing leverage despite identical local mass/COM/tensors. The
lower-arm visual/collision placement is shifted 0.65 mm to align with the
corrected wrist origin; inherited PAR6 local inertials are retained.
The original export's
reversed axes are documented only as a historical source conversion.
Neither actuator calibration nor physical PAROL6 validation is established
by inheriting these parameters. Finite position gains and torque limits
remain existing estimates; see ACTUATION.md.
''', encoding='utf-8')
    # Preserve the existing PAR6 ROS 1 package convention in the new package.
    for name in ['package.xml', 'CMakeLists.txt']:
        text = (PAR6_URDF/name).read_text().replace('par6_description', 'parol6_description').replace('PAR6', 'PAROL6').replace('par6_', 'parol6_')
        if name == 'CMakeLists.txt':
            text = text.replace('DIRECTORY config launch meshes docs previews', 'DIRECTORY config launch meshes docs previews scripts')
        (URDF/name).write_text(text, encoding='utf-8')
    for directory in ['config', 'launch']:
        for path in (PAR6_URDF/directory).rglob('*'):
            if path.is_file():
                destination = URDF/path.relative_to(PAR6_URDF)
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_text(path.read_text().replace('par6_description', 'parol6_description').replace('par6_', 'parol6_'), encoding='utf-8')
    (URDF/'scripts').mkdir(exist_ok=True)
    shutil.copyfile(ROOT/'tools/ros_urdf.py', URDF/'scripts/ros_urdf.py')
    param = '  <param name="robot_description" command="python3 &quot;$(find parol6_description)/scripts/ros_urdf.py&quot; &quot;$(find parol6_description)/parol6_$(arg end_effector).urdf&quot;" />\n'
    (URDF/'launch/display.launch').write_text('''<launch>
  <arg name="end_effector" default="msg" doc="flange, msg, msg_tpu_glue, ssg48, ssg48_stock, pneumatic" />
'''+param+'''  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" />
</launch>
''', encoding='utf-8')
    (URDF/'launch/gazebo.launch').write_text('''<launch>
  <arg name="end_effector" default="msg" doc="flange, msg, msg_tpu_glue, ssg48, ssg48_stock, pneumatic" />
  <include file="$(find gazebo_ros)/launch/empty_world.launch" />
'''+param+'''  <node name="spawn_parol6" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model parol6_$(arg end_effector)" output="screen" />
</launch>
''', encoding='utf-8')
    shutil.copytree(PAR6_MJCF/'bench', MJCF/'bench', dirs_exist_ok=True)


def build():
    settings = config()
    mounting = wrist_mount_geometry()
    for relative, expected in settings['original_sha256'].items():
        if hashlib.sha256((ORIGINAL/relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f'PAROL6 source changed: {relative}; review and explicitly re-import it')
    URDF.mkdir(parents=True, exist_ok=True)
    MJCF.mkdir(parents=True, exist_ok=True)
    partitions = visual_parts(settings)
    source = ET.parse(ORIGINAL/'urdf/parol6.urdf').getroot()
    for profile in PROFILES:
        tree = ET.parse(PAR6_URDF/f'par6_{profile}.urdf')
        tree.getroot().set('name', f'parol6_{profile}')
        replace_urdf_arm(tree.getroot(), source, settings)
        copy_urdf_assets(tree.getroot(), settings)
        write_xml(tree, URDF/f'parol6_{profile}.urdf')
        tree = ET.parse(PAR6_MJCF/f'par6_{profile}.xml')
        tree.getroot().set('model', f'PAROL6_{profile.upper()}')
        replace_native_arm(tree.getroot(), source, settings, profile)
        copy_native_assets(tree.getroot(), settings)
        write_xml(tree, MJCF/f'parol6_{profile}.xml')
        scene = (PAR6_MJCF/f'scene_{profile}.xml').read_text().replace('PAR6', 'PAROL6').replace('par6_', 'parol6_')
        (MJCF/f'scene_{profile}.xml').write_text(scene, encoding='utf-8')
        mujoco.MjModel.from_xml_path(str(MJCF/f'scene_{profile}.xml'))
    (MJCF/'scene.xml').write_text('<mujoco model="PAROL6 default"><include file="scene_msg.xml"/></mujoco>\n', encoding='utf-8')
    metadata_and_support()
    report = {'source_sha256': settings['original_sha256'], 'visual_partitions': partitions,
              'profiles': list(PROFILES), 'dynamics': settings['dynamics_policy'],
              'joint_policy': settings['joint_policy'], 'par6_from_parol6_joint_signs': PAR6_FROM_PAROL6.tolist(),
              'source_from_parol6_joint_signs': SOURCE_FROM_PAROL6.tolist(),
              'actuation_policy': settings['actuation_policy'], 'geometry': settings['geometry'],
              'mounting_geometry': mounting}
    (ROOT/'reports/parol6_build.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    for package in [MJCF, URDF]:
        (documentation(package)/'PNEUMATIC.md').write_bytes((documentation(PAR6_MJCF)/'PNEUMATIC.md').read_bytes())
    print(f'Built all {len(PROFILES)} PAROL6 URDF/MuJoCo profiles from included sources.', flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--import-source', type=Path, help='Explicit maintainer capture of an exported parol6 folder; normal builds need no external path')
    args = parser.parse_args()
    if args.import_source is not None:
        import_source(args.import_source)
    build()
