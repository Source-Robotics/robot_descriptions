# SPDX-License-Identifier: Apache-2.0
"""Shared, build-time appearance settings. No runtime dependency is introduced."""

import copy
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'source/appearance/colors.json'
COLOR_KEYS = {
    'structure', 'motors', 'camera_lens', 'msg_solid_jaws',
    'msg_soft_holders', 'msg_soft_inserts', 'ssg48_solid_jaws',
    'ssg48_fin_ray_jaws', 'pneumatic_body', 'pneumatic_jaws',
}
OVERRIDES = {
    'j1_motor': 'motors', 'j2_motor': 'motors', 'j3_motor': 'motors',
    'msg_motor': 'motors', 'msg_body': 'structure', 'ssg48_body': 'structure',
    'flange': 'structure', 'par6_structure': 'structure',
    'parol6_structure': 'structure', 'par6_panel': 'structure',
}
BADGE_KEYS = {'background', 'emblem', 'symbol', 'wordmark', 'robot_name', 'tagline', 'divider'}
MATERIALS = {
    **{f'j{i}_actuator_black': f'j{i}_motor' for i in range(1, 4)},
    'msg_camera_lens_black': 'camera_lens', 'msg_gripper_motor_black': 'msg_motor',
    'msg_jaw_black': 'msg_soft_holders', 'msg_jaw_yellow': 'msg_soft_inserts',
    'ssg48_jaw_material': 'ssg48_solid_jaws',
}
ARM_LINKS = {'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link',
             'wrist_1_link', 'wrist_2_link', 'wrist_3_link'}


def rgba(value):
    """Accept #RRGGBB or opaque [red, green, blue, 1] with 0..1 channels."""
    if isinstance(value, str) and re.fullmatch(r'#[0-9a-fA-F]{6}', value):
        return tuple(int(value[i:i+2], 16)/255 for i in (1, 3, 5))+(1.,)
    if (not isinstance(value, list) or len(value) != 4 or
            any(isinstance(x, bool) or not isinstance(x, (int, float)) or
                not math.isfinite(x) or not 0 <= x <= 1 for x in value) or value[3] != 1):
        raise ValueError(f'Expected #RRGGBB or opaque [R, G, B, 1], got {value!r}')
    return tuple(float(x) for x in value)


def numbers(value):
    return ' '.join(f'{x:.9g}' for x in rgba(value))


def hex_color(value):
    return '#'+''.join(f'{round(x*255):02X}' for x in rgba(value)[:3])


def validate(palette):
    if not isinstance(palette, dict) or set(palette) != {'schema_version', 'colors', 'overrides', 'badge'}:
        raise ValueError('Palette requires schema_version, colors, overrides and badge only')
    if type(palette['schema_version']) is not int or palette['schema_version'] != 1:
        raise ValueError('Unsupported color palette schema_version')
    for section, required in [('colors', COLOR_KEYS), ('badge', BADGE_KEYS)]:
        if not isinstance(palette[section], dict) or set(palette[section]) != required:
            raise ValueError(f'{section}: expected keys {sorted(required)}')
    if not isinstance(palette['overrides'], dict) or set(palette['overrides'])-OVERRIDES.keys():
        raise ValueError(f'overrides: supported parts are {sorted(OVERRIDES)}')
    for section in ['colors', 'overrides', 'badge']:
        for key, value in palette[section].items():
            try:
                rgba(value)
            except ValueError as error:
                raise ValueError(f'{section}.{key}: {error}') from error
    return palette


def load(path=None):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'Duplicate color setting: {key}')
            result[key] = value
        return result
    return validate(json.loads(Path(path or CONFIG).read_text(encoding='utf-8'), object_pairs_hook=unique))


def color(palette, part):
    return palette['overrides'].get(part, palette['colors'][OVERRIDES.get(part, part)])


def badge_config(config, palette):
    """Keep SVG geometry/UV/texture settings; replace only its palette."""
    config = copy.deepcopy(config)
    # The panel follows a PAR6 structure override unless explicitly overridden.
    panel = palette['overrides'].get('par6_panel', color(palette, 'par6_structure'))
    config['panel_color'] = hex_color(panel)
    config['colors'] = {key: hex_color(value) for key, value in palette['badge'].items()}
    return config


def part_for(body, visual, family, profile):
    material = visual.get('material') if visual.tag == 'geom' else visual.find('material')
    material = material if isinstance(material, str) else material.get('name') if material is not None else None
    if material in MATERIALS:
        return MATERIALS[material]
    if body in ARM_LINKS:
        return f'{family}_structure'
    if body == 'flange_link':
        return 'flange'
    if body == 'msg_gripper_link':
        return 'msg_body'
    if body == 'ssg48_gripper_link':
        return 'ssg48_body'
    if body == 'pneumatic_gripper_link':
        return 'pneumatic_body'
    if body in {'left_finger_link', 'right_finger_link'}:
        return {'msg': 'msg_solid_jaws', 'msg_tpu_glue': 'msg_soft_holders',
                'ssg48': 'ssg48_solid_jaws', 'ssg48_stock': 'ssg48_fin_ray_jaws',
                'pneumatic': 'pneumatic_jaws'}[profile]
    raise ValueError(f'Unmapped robot visual: {family}/{profile}/{body}/{visual.attrib}')


def physical_definition(root):
    """All XML except allowed appearance fields; used to guard every rewrite."""
    root = copy.deepcopy(root)
    if root.tag == 'robot':
        for visual in root.findall('link/visual'):
            for material in visual.findall('material'):
                visual.remove(material)
    else:
        for material in root.findall('asset/material'):
            material.attrib.pop('rgba', None)
        for geom in root.findall('worldbody//geom'):
            geom.attrib.pop('rgba', None)
    def canonical(element):
        return (element.tag, tuple(sorted(element.attrib.items())),
                (element.text or '').strip(), tuple(canonical(c) for c in element))
    return canonical(root)


def apply_tree(root, family, profile, palette):
    """Only existing visual RGBA fields change; preserve all other model data."""
    before = physical_definition(root)
    if root.tag == 'mujoco':
        for material in root.findall('asset/material'):
            if material.get('name') in MATERIALS:
                material.set('rgba', numbers(color(palette, MATERIALS[material.get('name')])))
        for body in root.findall('worldbody//body'):
            for geom in body.findall('geom'):
                # Textured/material-backed visuals are handled by their asset.
                # Hidden original meshes/contact proxies must remain invisible.
                if geom.get('material') or geom.get('group') == '3':
                    continue
                value = geom.get('rgba')
                if value is None or float(value.split()[-1]) == 0:
                    continue
                geom.set('rgba', numbers(color(palette, part_for(body.get('name'), geom, family, profile))))
    else:
        for link in root.findall('link'):
            for visual in link.findall('visual'):
                entry = visual.find('material/color')
                if entry is None:
                    # The PAR6 cover's DAE embeds the baked SVG texture.
                    mesh = visual.find('geometry/mesh')
                    if mesh is not None and mesh.get('filename', '').endswith('upper_arm_structure_badged.dae'):
                        continue
                    raise ValueError(f'Visual has no color: {family}/{profile}/{link.get("name")}')
                entry.set('rgba', numbers(color(palette, part_for(link.get('name'), visual, family, profile))))
    if physical_definition(root) != before:
        raise AssertionError('Color application modified a non-appearance field')
    return root


def apply_package(package, family, palette=None):
    palette = load() if palette is None else validate(palette)
    package = Path(package)
    native = any(package.glob(f'{family}_*.xml'))
    suffix = '.xml' if native else '.urdf'
    standalone = {'solid': 'msg' if family == 'msg' else 'ssg48',
                  'soft_tpu': 'msg_tpu_glue', 'fin_ray': 'ssg48_stock'}
    for path in sorted(package.glob(f'{family}_*{suffix}')):
        profile = path.stem.removeprefix(f'{family}_')
        profile = standalone[profile] if family in {'msg', 'ssg48'} else profile
        tree = ET.parse(path)
        before = ET.tostring(tree.getroot())
        apply_tree(tree.getroot(), family, profile, palette)
        if ET.tostring(tree.getroot()) != before:
            ET.indent(tree, space='  ')
            tree.write(path, encoding='utf-8', xml_declaration=True)
