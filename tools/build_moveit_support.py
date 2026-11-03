# SPDX-License-Identifier: Apache-2.0
"""Generate separate MoveIt 2 configuration packages from maintained models."""
from itertools import combinations
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_support.robot_description import ARM_TOOLS
from moveit_support.model_config import ARM_JOINTS

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'tools/moveit_support'
PROFILES = ('flange', 'msg', 'msg_tpu_glue', 'ssg48', 'ssg48_stock', 'pneumatic')


def json_file(path, data):
    # JSON is a strict YAML subset, accepted by both ROS and PyYAML. Keeping
    # generation stdlib-only avoids adding CAD build dependencies for ROS files.
    path.write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8', newline='\n')


def rigid_collision_pairs(root):
    """Disable only rigid-island and directly connected moving-island pairs.

    No random 'never collides' guesses and no blanket gripper/arm exemptions.
    Opposing fingers belong to different, nonadjacent moving islands.
    """
    islands = {l.get('name'): l.get('name') for l in root.findall('link')}
    def find(name):
        while islands[name] != name:
            name = islands[name]
        return name
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            islands[find(joint.find('child').get('link'))] = find(joint.find('parent').get('link'))
    adjacent = {frozenset([find(j.find('parent').get('link')), find(j.find('child').get('link'))])
                for j in root.findall('joint') if j.get('type') != 'fixed'}
    links = [l.get('name') for l in root.findall('link') if l.find('collision') is not None]
    pairs = []
    for a, b in combinations(links, 2):
        if find(a) == find(b):
            pairs.append((a, b, 'Rigid assembly'))
        elif frozenset([find(a), find(b)]) in adjacent:
            pairs.append((a, b, 'Adjacent'))
    return pairs


def profile_data(family, root):
    reference = json.loads((ROOT/'source/dynamics/rcb_reference.json').read_text())
    bounds = {}
    for index, name in enumerate(ARM_JOINTS):
        joint = root.find(f'joint[@name="{name}"]')
        if family == 'par6':
            limits = reference['variants']['flange']['software_joint_limits_rad']
            offset = reference['rcb_from_mj_offset_rad'][index]
            bounds[name] = [float(limits[0][index]-offset), float(limits[1][index]-offset)]
        else:
            limit = joint.find('limit')
            bounds[name] = [float(limit.get('lower')), float(limit.get('upper'))]
    gripper = root.find('joint[@name="left_finger_joint"]') is not None
    initial = dict.fromkeys(ARM_JOINTS, 0.)
    states = {'home': dict(initial), 'ready': dict(zip(ARM_JOINTS, [0., -.4, .6, 0., .5, 0.]))}
    if gripper:
        for name in ['left_finger_joint', 'right_finger_joint']:
            limit = root.find(f'joint[@name="{name}"]/limit')
            bounds[name] = [float(limit.get('lower')), float(limit.get('upper'))]
        pneumatic = root.find('link[@name="pneumatic_gripper_link"]') is not None
        lower, upper = bounds['left_finger_joint']
        opened, closed = (upper, lower) if pneumatic else (upper, lower)
        # The original MSG meshes overlap 0.8 mm at the mathematical endpoint.
        # Keep that model limit, but use a 0.5 mm/finger stand-off for the
        # empty-gripper closed planning preset so collision checking stays on.
        if root.find('link[@name="msg_gripper_link"]') is not None:
            closed += .0005
        states.update(open={'left_finger_joint': opened}, closed={'left_finger_joint': closed})
        initial['left_finger_joint'] = opened
        mimic = root.find('joint[@name="right_finger_joint"]/mimic')
        initial['right_finger_joint'] = opened*float(mimic.get('multiplier', '1'))+float(mimic.get('offset', '0'))
    return dict(family=family, has_gripper=gripper, tcp='tcp', position_limits=bounds,
                position_limit_source='Captured RCB limits minus RCB/package angle offsets' if family == 'par6'
                    else 'Maintained PAROL6 URDF finite ranges, including the stored J6 range',
                initial_positions=initial, named_states=states)


def semantic(root, data):
    srdf = ET.Element('robot', name=root.get('name'))
    ET.SubElement(srdf, 'virtual_joint', name='world_joint', type='fixed', parent_frame='world', child_link='base_link')
    group = ET.SubElement(srdf, 'group', name='arm')
    ET.SubElement(group, 'chain', base_link='base_link', tip_link='tcp')
    if data['has_gripper']:
        group = ET.SubElement(srdf, 'group', name='gripper')
        for name in ['left_finger_joint', 'right_finger_joint']:
            ET.SubElement(group, 'joint', name=name)
        ET.SubElement(srdf, 'end_effector', name='gripper', parent_link='tcp', group='gripper', parent_group='arm')
    for name, values in data['named_states'].items():
        group = 'gripper' if name in ('open', 'closed') else 'arm'
        state = ET.SubElement(srdf, 'group_state', name=name, group=group)
        for joint, value in values.items():
            ET.SubElement(state, 'joint', name=joint, value=str(value))
    for a, b, reason in rigid_collision_pairs(root):
        ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b, reason=reason)
    ET.indent(srdf, space='  ')
    return ET.tostring(srdf, encoding='unicode')+'\n'


def build():
    version = ET.parse(ROOT/'urdf/par6_description/package.xml').findtext('version')
    for family in ('par6', 'parol6'):
        name = family+'_moveit_config'
        package = ROOT/'moveit'/name
        for directory in ['config/profiles', 'launch', 'scripts', 'docs']:
            (package/directory).mkdir(parents=True, exist_ok=True)
        for profile in PROFILES:
            root = ET.parse(ROOT/f'urdf/{family}_description/{family}_{profile}.urdf').getroot()
            data = profile_data(family, root)
            json_file(package/f'config/profiles/{profile}.json', data)
            (package/f'config/profiles/{profile}.srdf').write_text(semantic(root, data), encoding='utf-8', newline='\n')
        shutil.copyfile(SOURCE/'model_config.py', package/'scripts/model_config.py')
        (package/'launch/demo.launch.py').write_text((SOURCE/'demo.launch.py').read_text().replace('@FAMILY@', family), encoding='utf-8', newline='\n')
        for filename in ['kinematics.yaml', 'ompl_planning.yaml', 'ompl_humble.yaml', 'planning_parameters.yaml', 'joint_limits.yaml', 'moveit.rviz']:
            shutil.copyfile(SOURCE/filename, package/'config'/filename)
        dependencies = [family+'_description', 'ament_index_python', 'launch', 'launch_ros', 'python3-yaml',
            'robot_state_publisher', 'tf2_ros', 'rviz2', 'moveit_ros_move_group', 'moveit_ros_visualization',
            'moveit_kinematics', 'moveit_planners_ompl', 'moveit_simple_controller_manager',
            'controller_manager', 'hardware_interface', 'joint_state_broadcaster', 'joint_trajectory_controller']
        manifest = ET.Element('package', format='3')
        for tag, value in [('name', name), ('version', version), ('description', f'MoveIt 2 planning and mock execution for Source Robotics {family.upper()}.'),
                           ('maintainer', 'Source Robotics'), ('license', 'CC-BY-4.0'), ('license', 'Apache-2.0'), ('buildtool_depend', 'ament_cmake')]:
            element = ET.SubElement(manifest, tag); element.text = value
            if tag == 'maintainer': element.set('email', 'petar@source-robotics.com')
        for dependency in dependencies: ET.SubElement(manifest, 'exec_depend').text = dependency
        ET.SubElement(ET.SubElement(manifest, 'export'), 'build_type').text = 'ament_cmake'
        ET.indent(manifest, space='  ')
        (package/'package.xml').write_text(ET.tostring(manifest, encoding='unicode')+'\n', encoding='utf-8')
        (package/'CMakeLists.txt').write_text(f'''# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project({name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch scripts docs DESTINATION share/${{PROJECT_NAME}})
install(FILES README.md LICENSE DESTINATION share/${{PROJECT_NAME}})
ament_package()
''', encoding='utf-8')
        shutil.copyfile(ROOT/'LICENSE', package/'LICENSE')
        shutil.copyfile(ROOT/'LICENSES/Apache-2.0.txt', package/'docs/LICENSE-CODE.txt')
        shutil.copyfile(ROOT/'docs/MOVEIT2.md', package/'docs/MOVEIT2.md')
        (package/'README.md').write_text(f'''# {family.upper()} MoveIt 2

Motion planning and simulated trajectory execution for all six {family.upper()}
tool/jaw configurations. Requires the matching `{family}_description` package
from the same release. No meshes or physical model data are duplicated here.

Follow [installation and the RViz walkthrough](docs/MOVEIT2.md), then launch:

```bash
ros2 launch {name} demo.launch.py end_effector:=msg jaws:=soft_tpu
```

Choose the `arm` group and `tcp` tip in RViz; select **Plan**, then **Execute**.
Execution uses `mock_components/GenericSystem`, which follows joint commands
ideally. It tests planning/control integration without simulating loads,
contact forces, motor dynamics or the real RCB controller.

Tool options: `flange`, `msg`, `ssg48`, `pneumatic`. Electric jaw options:
MSG `solid` / `soft_tpu`; SSG48 `solid` / `fin_ray`. Omit `jaws` for flange
and pneumatic. `rviz:=false` runs the same planning/execution stack headlessly.

## Files

- `config/profiles/*.srdf`: groups, TCP, named poses, collision exemptions.
- `config/profiles/*.json`: model selection, initial positions and joint ranges.
- `config/joint_limits.yaml`: editable demo speeds, accelerations and effort placeholders.
- `config/kinematics.yaml`: TCP inverse-kinematics solver.
- `config/ompl*.yaml`: Humble/Jazzy planning pipelines.
- `config/planning_parameters.yaml`: execution and planning-scene settings.
- `config/moveit.rviz`: saved MotionPlanning view.
- `launch/demo.launch.py`: selection, planning, mock controllers and RViz.
- `scripts/model_config.py`: in-memory planning/controller adapter.

Controller settings are assembled for the selected tool, so flange-only has
no gripper controller and mimic fingers are never independently commanded.
The description package's saved URDFs/geometry stay unchanged. See the guide
for joint-limit sources and the MSG closed-preset stand-off.

Model/semantic data and documentation: **CC BY 4.0**, copyright 2026 Source
Robotics; [source](https://github.com/Source-Robotics/robot_descriptions).
Launch/support code and CMake: **Apache-2.0**, see [code license](docs/LICENSE-CODE.txt).
''', encoding='utf-8')
    print('Generated PAR6 and PAROL6 MoveIt 2 configurations (12 tool variants).', flush=True)


if __name__ == '__main__':
    build()
