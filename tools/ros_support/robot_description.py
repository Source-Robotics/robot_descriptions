# SPDX-License-Identifier: Apache-2.0
"""ROS-independent selection and resource resolution for the installed URDFs."""
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET

ARM_TOOLS = ('flange', 'msg', 'ssg48', 'pneumatic', 'msg_tpu_glue', 'ssg48_stock')
JAW_OPTIONS = {'msg': ('solid', 'soft_tpu'), 'ssg48': ('solid', 'fin_ray')}


def select_model(family, end_effector='msg', jaws='default'):
    if family in JAW_OPTIONS:
        jaws = 'solid' if jaws == 'default' else jaws
        if jaws not in JAW_OPTIONS[family]:
            raise ValueError(f'{family}: jaws must be one of {JAW_OPTIONS[family]}')
        return f'{family}_{jaws}.urdf'
    if family not in ('par6', 'parol6'):
        raise ValueError(f'Unknown product: {family}')
    if end_effector not in ARM_TOOLS:
        raise ValueError(f'end_effector must be one of {ARM_TOOLS}')
    aliases = {'msg_tpu_glue': ('msg', 'soft_tpu'), 'ssg48_stock': ('ssg48', 'fin_ray')}
    if end_effector in aliases:
        tool, implied = aliases[end_effector]
        if jaws not in ('default', implied):
            raise ValueError(f'{end_effector} already selects {implied} jaws; remove the conflicting jaws argument')
        end_effector, jaws = tool, implied
    if end_effector in ('flange', 'pneumatic'):
        if jaws != 'default':
            raise ValueError(f'{end_effector} has no selectable jaws; omit the jaws argument')
        profile = end_effector
    else:
        jaws = 'solid' if jaws == 'default' else jaws
        if jaws not in JAW_OPTIONS[end_effector]:
            raise ValueError(f'{end_effector}: jaws must be one of {JAW_OPTIONS[end_effector]}')
        profile = {('msg', 'solid'): 'msg', ('msg', 'soft_tpu'): 'msg_tpu_glue',
                   ('ssg48', 'solid'): 'ssg48', ('ssg48', 'fin_ray'): 'ssg48_stock'}[end_effector, jaws]
    return f'{family}_{profile}.urdf'


def separate_rviz_visuals(root):
    """Work around RViz applying the first visual material to an entire link.

    Move additional visuals to massless fixed children. Physical links, their
    inertials/collisions and all existing joint/frame transforms stay intact.
    """
    existing = {element.get('name') for element in root if element.tag in ('link', 'joint')}
    for link in root.findall('link'):
        visuals = link.findall('visual')
        for index, visual in enumerate(visuals):
            material = visual.find('material')
            if material is not None and not material.get('name'):
                material.set('name', f'{link.get("name")}_rviz_material_{index}')
        for index, visual in enumerate(visuals[1:], 1):
            name = f'{link.get("name")}_rviz_visual_{index}'
            if name in existing or name+'_fixed' in existing:
                raise ValueError(f'RViz visual frame conflicts with an existing name: {name}')
            existing.update([name, name+'_fixed'])
            child = ET.SubElement(root, 'link', name=name)
            link.remove(visual); child.append(visual)
            joint = ET.SubElement(root, 'joint', name=name+'_fixed', type='fixed')
            ET.SubElement(joint, 'parent', link=link.get('name'))
            ET.SubElement(joint, 'child', link=name)
            ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')


def load_description(path, package, rviz_materials=False):
    """Resolve asset URIs in memory; never rewrite model or physical data."""
    path = Path(path)
    root = ET.parse(path).getroot()
    for entry in [*root.findall('.//mesh'), *root.findall('.//material/texture')]:
        filename = entry.get('filename')
        if not filename or '://' in filename:
            continue
        relative = PurePosixPath(filename.replace('\\', '/'))
        target = (path.parent/relative).resolve()
        if relative.is_absolute() or not target.is_relative_to(path.parent.resolve()):
            raise ValueError(f'Asset escapes the installed package: {filename}')
        if not target.is_file():
            raise FileNotFoundError(f'Missing model asset: {target}')
        entry.set('filename', f'package://{package}/{relative.as_posix()}')
    if rviz_materials:
        separate_rviz_visuals(root)
    return ET.tostring(root, encoding='unicode')
