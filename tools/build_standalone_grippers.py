"""Export independent MSG/SSG48 packages from the maintained tool subtrees.

The flange is the fixed mounting frame. Preserve all tool-relative transforms,
mass properties, finger joints, contacts, materials and camera definitions.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
from model_reference import ROOT

GRIPPERS = {
    'msg': {'label': 'MSG', 'mass': .380, 'travel': .0525,
            'profiles': [('solid', 'msg', 'Solid jaws'),
                         ('soft_tpu', 'msg_tpu_glue', 'Soft TPU jaws')]},
    'ssg48': {'label': 'SSG48', 'mass': .440, 'travel': .0235,
              'profiles': [('solid', 'ssg48', 'Solid jaws'),
                           ('fin_ray', 'ssg48_stock', 'Fin-ray jaws')]},
}


from package_layout import documentation, previews

def write_xml(path, root):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def copy_asset(source, source_package, destination, relative):
    source = source.resolve()
    assert source.is_relative_to(source_package.resolve()) and source.is_file(), source
    target = destination / relative
    assert target.resolve().is_relative_to(destination.resolve()), target
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)


def native_model(family, profile, destination):
    package = ROOT / 'mujoco/par6'
    source = ET.parse(package / f'par6_{profile}.xml').getroot()
    flange = copy.deepcopy(source.find('.//body[@name="flange"]'))
    flange.set('pos', '0 0 0')
    result = ET.Element('mujoco', model=f'{GRIPPERS[family]["label"]} standalone gripper')
    ET.SubElement(result, 'compiler', angle='radian', autolimits='true')
    result.append(copy.deepcopy(source.find('option')))
    defaults = ET.SubElement(result, 'default')
    classes = {x.get('class') for x in flange.iter() if x.get('class')}
    for entry in source.find('default'):
        if entry.get('class') in classes:
            defaults.append(copy.deepcopy(entry))
    asset = ET.SubElement(result, 'asset')
    wanted = set()
    for entry in flange.iter():
        for kind in ['mesh', 'material', 'texture']:
            if entry.get(kind):
                wanted.add((kind, entry.get(kind)))
    catalog = {(x.tag, x.get('name')): x for x in source.find('asset')}
    pending = list(wanted)
    while pending:
        key = pending.pop()
        entry = catalog[key]
        for kind in ['material', 'texture']:
            dependency = (kind, entry.get(kind))
            if entry.get(kind) and dependency not in wanted:
                wanted.add(dependency); pending.append(dependency)
    compiler = source.find('compiler')
    for original in source.find('asset'):
        if (original.tag, original.get('name')) not in wanted:
            continue
        entry = copy.deepcopy(original)
        if entry.get('file'):
            directory = compiler.get('meshdir', '') if entry.tag == 'mesh' else compiler.get('texturedir', '')
            path = (package / directory / entry.get('file')).resolve()
            relative = Path('assets') / path.relative_to(package / 'assets')
            copy_asset(path, package, destination, relative)
            entry.set('file', relative.as_posix())
        asset.append(entry)
    world = ET.SubElement(result, 'worldbody')
    world.append(flange)
    # A fixed tool is welded to world; explicitly retain the adjacent-body
    # filtering provided by its moving arm parent in the robot assembly.
    contact = ET.SubElement(result, 'contact')
    for side in ['left', 'right']:
        ET.SubElement(contact, 'exclude', body1=f'{family}_gripper_link', body2=f'{side}_finger_link')
    joints = {x.get('name') for x in flange.iter('joint')}
    equality = ET.SubElement(result, 'equality')
    for entry in source.find('equality'):
        if {entry.get('joint1'), entry.get('joint2')} <= joints:
            equality.append(copy.deepcopy(entry))
    actuators = ET.SubElement(result, 'actuator')
    for entry in source.find('actuator'):
        if entry.get('joint') in joints:
            actuators.append(copy.deepcopy(entry))
    travel = GRIPPERS[family]['travel']
    keys = ET.SubElement(result, 'keyframe')
    ET.SubElement(keys, 'key', name='open', qpos='0 0', ctrl='0')
    ET.SubElement(keys, 'key', name='closed', qpos=f'{-travel} {travel}', ctrl=str(-travel))
    return result


def urdf_model(family, profile, destination):
    package = ROOT / 'urdf/par6_description'
    source = ET.parse(package / f'par6_{profile}.urdf').getroot()
    names = {'flange'}
    selected = []
    pending = list(source.findall('joint'))
    while pending:
        found = [x for x in pending if x.find('parent').get('link') in names]
        if not found:
            break
        for joint in found:
            names.add(joint.find('child').get('link'))
            selected.append(joint)
            pending.remove(joint)
    assert f'{family}_gripper_link' in names
    result = ET.Element('robot', name=f'{family}_gripper')
    for link in source.findall('link'):
        if link.get('name') in names:
            result.append(copy.deepcopy(link))
    for joint in selected:
        result.append(copy.deepcopy(joint))
    joints = {x.get('name') for x in selected}
    for transmission in source.findall('transmission'):
        if all(x.get('name') in joints for x in transmission.findall('joint')):
            result.append(copy.deepcopy(transmission))
    # Preserve any global materials referenced by the retained visuals.
    for material in source.findall('material'):
        if material.get('name') in {x.get('name') for x in result.findall('.//visual/material')}:
            result.append(copy.deepcopy(material))
    for mesh in result.findall('.//mesh'):
        relative = Path(mesh.get('filename'))
        copy_asset(package / relative, package, destination, relative)
    return result


def metadata(family, native, version):
    config = GRIPPERS[family]
    variants = []
    for suffix, _, label in config['profiles']:
        variants.append({'id': suffix.replace('_', '-'), 'label': f'{config["label"]} - {label}',
                         'entry': f'scene_{suffix}.xml' if native else f'{family}_{suffix}.urdf',
                         'endEffectorBodies': [f'{family}_gripper_link'],
                         'gripperActuators': ['gripper_actuator'] if native else []})
        if (ROOT/f'source/grippers/previews/{family}_{suffix}.png').is_file():
            variants[-1]['preview'] = f'previews/{family}_{suffix}.png'
    return {'version': 1, 'name': f'Source Robotics {config["label"]} gripper',
            'manufacturer': 'Source Robotics', 'category': 'gripper', 'license': 'Apache-2.0',
            'sourceUrl': 'https://github.com/Source-Robotics/robot_descriptions',
            'packageVersion': version, 'entry': variants[0]['entry'], 'dof': 1,
            'rootFrame': 'flange', 'tcpSites': ['tcp'], 'armActuators': [],
            'gripperActuators': variants[0]['gripperActuators'],
            'endEffectorBodies': [f'{family}_gripper_link'], 'variants': variants}


def readme(family, native):
    config = GRIPPERS[family]; label = config['label']
    rows = '\n'.join(f'| {name}{" (default)" if suffix == "solid" else ""} | '
                     f'`scene_{suffix}.xml` | `{family}_{suffix}.xml` |'
                     if native else f'| {name}{" (default)" if suffix == "solid" else ""} | `{family}_{suffix}.urdf` |'
                     for suffix, _, name in config['profiles'])
    table = '| Jaws | Viewer scene | Gripper model |\n| --- | --- | --- |' if native else '| Jaws | URDF entry file |\n| --- | --- |'
    command = (f'python -m mujoco.viewer --mjcf={family}/scene_solid.xml' if native else
               f'roslaunch {family}_description display.launch jaws:=solid')
    endpoint_note = '''For MSG, the opposing jaws touch about 0.4 mm per finger before the mathematical
joint-limit target. Position control stops against that contact; forcing both
qpos values to their exact closed limits produces the same 0.8 mm overlap as
the original mounted model. Use the actuator for physical closing motion.
''' if family == 'msg' else ''
    controls = (f'''The fixed mount is `flange`. The native model has two coupled slide joints
and one position actuator, `gripper_actuator`. Control is in meters:
`0` opens; `-{config['travel']}` closes. The `open` and `closed` keyframes
show those endpoints. Keep the joint equality enabled. To attach it to a moving
robot, mount the `flange` subtree in that robot rather than adding another arm.
The explicit housing/finger exclusions preserve the robot package's adjacent
body filtering when the standalone housing is fixed to world.
{endpoint_note}
''' if native else '''The URDF root link, `flange`, is the mounting face. Attach it to your robot
with a fixed joint. `left_finger_joint` drives the mechanism and
`right_finger_joint` follows its retained mimic relation. Your URDF consumer
must honor the mimic joint and exclude adjacent housing/finger collisions.
The supplied display launch is ROS 1; ROS 2 users can load the same URDF using
their own `robot_state_publisher`/RViz launch setup. URDF itself does not
implement the native MuJoCo position servo or contact solver.
''')
    return f'''# Source Robotics {label}: standalone {'MuJoCo' if native else 'URDF'} gripper

This self-contained package contains **only the {label} gripper**, with both jaw
options, its fixed mounting frame and configured TCP. All required meshes and
materials are included. Keep the whole folder together when moving or uploading it.

![{label} with Solid jaws](previews/{family}_solid.png)

{table}
{rows}

The default is Solid jaws. {'`scene.xml` is an alias for `scene_solid.xml`.' if native else 'The URDF filenames are at the package root and use relative mesh paths.'}

Detailed notes are in [docs/](docs/), model images in [previews/](previews/),
and geometry in `{'assets' if native else 'meshes'}/`. The model entry files
remain at the package root.

## Open the model

{'After extracting the ZIP, run this from its parent directory:' if native else 'Open the selected URDF in a viewer with the meshes directory alongside it. For ROS 1, place this package in catkin_ws/src, build and source the workspace, then run:'}

```bash
{command}
```

## Mount and finger controls

{controls}
Each finger travels {config['travel']*1000:g} mm. Both formats preserve their
corresponding arm-mounted tool's joint origins, axes, limits, coupling, collision
surfaces, materials, mass properties and {'camera definition' if family == 'msg' else 'TCP definition'}.
No arm, flange hardware, grasp object, table, mocap target or free joint is in
the gripper model itself. The optional viewer scene adds only lighting and floor.
`flange` is a massless frame; the 55.5 g robot flange is not part of this package.

## Physical properties and current limits

Complete gripper mass: **{config['mass']*1000:g} g**. Fixed assembly:
**{(config['mass']-.0786)*1000:g} g**; each complete moving assembly: **39.3 g**.
Tool COM and inertia remain the current unverified estimates. The two jaw
options share those measured mass targets; TPU/Fin-ray deformation is not
simulated. Actuation parameters and grip force are not hardware calibration.

The retained `tcp` is the operational control frame, not a newly measured
jaw-specific grasp center. Frame and inertia definitions are copied from the
maintained robot assembly; exporting does not infer new physical measurements.

See [validation notes](docs/VALIDATION.md) for the extraction checks and `LICENSE` for Apache-2.0.
'''


def build():
    version = ET.parse(ROOT / 'urdf/par6_description/package.xml').findtext('version')
    for family, config in GRIPPERS.items():
        for native in [True, False]:
            destination = ROOT / (f'mujoco/{family}' if native else f'urdf/{family}_description')
            destination.mkdir(parents=True, exist_ok=True)
            for suffix, profile, _ in config['profiles']:
                preview = ROOT/f'source/grippers/previews/{family}_{suffix}.png'
                if preview.is_file():
                    shutil.copyfile(preview,previews(destination)/preview.name)
                if native:
                    model = native_model(family, profile, destination)
                    write_xml(destination/f'{family}_{suffix}.xml', model)
                    scene = ET.Element('mujoco', model=f'{config["label"]} gripper viewer')
                    ET.SubElement(scene, 'include', file=f'{family}_{suffix}.xml')
                    ET.SubElement(scene, 'statistic', center='0 0 .045', extent='.22')
                    visual = ET.SubElement(scene,'visual')
                    ET.SubElement(visual,'headlight',ambient='.35 .35 .35',diffuse='.8 .8 .8',specular='.15 .15 .15')
                    ET.SubElement(visual,'global',azimuth='135',elevation='-24')
                    assets = ET.SubElement(scene,'asset')
                    ET.SubElement(assets,'texture',type='skybox',builtin='gradient',rgb1='.22 .28 .34',
                                  rgb2='.38 .46 .55',width='64',height='384')
                    ET.SubElement(assets,'material',name='floor_material',rgba='.19 .24 .3 1')
                    world = ET.SubElement(scene, 'worldbody')
                    ET.SubElement(world, 'light', pos='.2 -.2 .6', dir='0 0 -1')
                    ET.SubElement(world, 'geom', name='floor', type='plane', pos='0 0 -.04',
                                  size='.5 .5 .01', material='floor_material')
                    write_xml(destination/f'scene_{suffix}.xml', scene)
                    if suffix == 'solid':
                        write_xml(destination/'scene.xml', copy.deepcopy(scene))
                else:
                    write_xml(destination/f'{family}_{suffix}.urdf', urdf_model(family, profile, destination))
            (destination/'sim2bot.robot.json').write_text(json.dumps(metadata(family,native,version),indent=2)+'\n',encoding='utf-8')
            (destination/'README.md').write_text(readme(family,native),encoding='utf-8')
            shutil.copyfile(ROOT/'LICENSE',destination/'LICENSE')
            (documentation(destination)/'VALIDATION.md').write_text('''# Standalone gripper validation

Generated from the maintained PAR6 tool subtree, with the root at the physical
`flange` frame. Both arms use the same tool-relative definitions.
`tools/verify_standalone_grippers.py` checks native and URDF exports for both jaw
options: reachable links/joints, required assets, measured masses, unchanged
local inertia, finger travel and FK relative to the original flange.
Native checks also cover the actuator, equality, camera, internal contacts and
finite close/open simulation. Release ZIPs are extracted into temporary folders
and checked without referring back to robot-package assets.

MSG's original mathematical closed endpoint overlaps the opposing jaws by about
0.8 mm. The forced-position sweep verifies this contact matches the source;
the physical close/open cycle instead verifies stable jaw contact, less than
20 microns of opposing-jaw solver compression, and successful reopening.

These are model-preservation and simulation checks, not physical calibration.
''',encoding='utf-8')
            if not native:
                package = ET.Element('package', format='2')
                ET.SubElement(package,'name').text=f'{family}_description'
                ET.SubElement(package,'version').text=version
                ET.SubElement(package,'description').text=f'Standalone Source Robotics {config["label"]} gripper with interchangeable jaws.'
                ET.SubElement(package,'maintainer',email='petar@source-robotics.com').text='Source Robotics'
                ET.SubElement(package,'license').text='Apache-2.0'
                ET.SubElement(package,'buildtool_depend').text='catkin'
                for dependency in ['joint_state_publisher_gui','robot_state_publisher','rviz']:
                    ET.SubElement(package,'exec_depend').text=dependency
                write_xml(destination/'package.xml',package)
                (destination/'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.0.2)
project({family}_description)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch meshes docs previews DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}})
install(FILES README.md LICENSE sim2bot.robot.json {family}_solid.urdf {family}_{config['profiles'][1][0]}.urdf
        DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}})
''',encoding='utf-8')
                launch = ET.Element('launch')
                ET.SubElement(launch,'arg',name='jaws',default='solid',doc='solid, '+config['profiles'][1][0])
                ET.SubElement(launch,'param',name='robot_description',textfile=f'$(find {family}_description)/{family}_$(arg jaws).urdf')
                for pkg,node in [('joint_state_publisher_gui','joint_state_publisher_gui'),('robot_state_publisher','robot_state_publisher'),('rviz','rviz')]:
                    ET.SubElement(launch,'node',pkg=pkg,type=node,name=node)
                (destination/'launch').mkdir(exist_ok=True)
                write_xml(destination/'launch/display.launch',launch)
    print('Exported four independent MSG/SSG48 packages, with both jaw options.',flush=True)


if __name__ == '__main__':
    argparse.ArgumentParser(description=__doc__).parse_args()
    build()
