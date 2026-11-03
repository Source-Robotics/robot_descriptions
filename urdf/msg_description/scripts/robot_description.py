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


def load_description(path, package):
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
    return ET.tostring(root, encoding='unicode')
