"""Rebuild actuator and MSG gripper visual-only partitions from original STLs.

Run with Python, numpy and scipy. Original triangle records, including normals,
are copied verbatim; the source collision meshes are never rewritten.
"""

from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'urdf/par6_description'
MJCF = ROOT / 'mujoco/par6'
BLACK = '0.06 0.06 0.065 1'
RECORD = np.dtype([('normal', '<f4', (3,)), ('vertex', '<f4', (3, 3)), ('attr', '<u2')])

# Component IDs are sorted by descending triangle count, with source-order ties.
# The hashes guard these assignments against changes to the original CAD exports.
SPLITS = {
    'shoulder': {
        'sha256': '5ecd33e6048df8757997827dd265acda2f1672294a3bcb985d9ffd75de35747e',
        'mjcf_sha256': '4b774a3b0902aa79c10e799201ef72c554bd0eacfc17b144e992a81fa37d254e',
        'link': 'shoulder_link',
        'components': 32,
        'parts': {
            'j1_actuator_visual': [0, 1, 4, 7, 8, 9, 10, 18, 20, 21, 22, 23, 28],
            'j2_actuator_visual': [2, 3, 5, 11, 12, 13, 14, 15, 19, 24, 25, 26, 27, 29],
            'shoulder_structure_visual': [6, 16, 17, 30, 31],
        },
    },
    'upper_arm': {
        'sha256': '1f08286e5e11b4d462646a06212d1d29e757c6e92778b74c71c0d50e43f98988',
        'mjcf_sha256': '3ffa7ed6e1836216f16813d3ce186c6d086e5133033db844477ea48feb4ea7c2',
        'link': 'upper_arm_link',
        'components': 18,
        'parts': {
            'j3_actuator_visual': [0, 1, 3, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 17],
            'upper_arm_structure_visual': [2, 4, 10, 16],
        },
    },
    'msg_gripper': {
        'sha256': '3027b14dbade6431bd8ccd9c87cc8496500062bf68891c718ee9b29fdea4cb6c',
        'mjcf_sha256': '18cfd48fbc4544cd7b3a44805c338979170380996505b4e3391e8e3c94c8a67a',
        'urdf_mesh': 'meshes/msg/gripper.STL',
        'urdf_visual_dir': 'meshes/msg/visual',
        'mjcf_mesh': 'gripper_MSG.STL',
        'mjcf_mesh_name': 'gripper',
        'variants': ['msg'],
        'link': 'msg_gripper_link',
        'components': 27,
        'parts': {
            'msg_camera_lens_visual': [3, 17],
            'msg_gripper_motor_visual': [5, 6, 7, 9, 10, 11, 12, 14, 16, 23],
            'msg_gripper_structure_visual': [0, 1, 2, 4, 8, 13, 15, 18, 19, 20, 21, 22, 24, 25, 26],
        },
        'materials': {
            'msg_camera_lens_visual': 'msg_camera_lens_black',
            'msg_gripper_motor_visual': 'msg_gripper_motor_black',
        },
    },
}


def variant_specs(variant):
    variant={'msg_tpu_glue':'msg','ssg48_stock':'ssg48'}.get(variant,variant)
    return {source: spec for source, spec in SPLITS.items()
            if variant in spec.get('variants', ['flange', 'msg', 'ssg48'])}


def source_paths(source, spec):
    return (URDF / spec.get('urdf_mesh', f'meshes/common/{source}.STL'),
            MJCF / 'assets' / spec.get('mjcf_mesh', f'{source}.STL'))


def visual_dir(spec):
    return spec.get('urdf_visual_dir', 'meshes/common/visual')


def materials(spec):
    return spec.get('materials', {name: name[:2] + '_actuator_black'
                                  for name in spec['parts'] if '_actuator_' in name})


def read_parts(path):
    blob = path.read_bytes()
    n = int.from_bytes(blob[80:84], 'little')
    if len(blob) != 84 + 50 * n:
        raise ValueError(f'Not a binary STL: {path}')
    records = np.frombuffer(blob, RECORD, n, 84)
    # Merge coordinates only to discover components; emitted records stay exact.
    _, inverse = np.unique(
        np.round(records['vertex'].reshape(-1, 3) * 1e7).astype('int64'),
        axis=0, return_inverse=True,
    )
    rows = np.repeat(np.arange(n), 3)
    cols = n + inverse
    count = n + inverse.max() + 1
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(count, count))
    _, labels = connected_components(graph, directed=False)
    groups = [np.where(labels[:n] == label)[0] for label in np.unique(labels[:n])]
    groups.sort(key=len, reverse=True)
    return blob, records, groups


def generate_meshes():
    report = {'description': 'Exact visual-only partitions; original meshes remain collision assets.', 'sources': {}}
    for source, spec in SPLITS.items():
        path, mj_path = source_paths(source, spec)
        blob, records, groups = read_parts(path)
        if hashlib.sha256(blob).hexdigest() != spec['sha256']:
            raise ValueError(f'Source mesh changed; inspect components before rebuilding {path}')
        if len(groups) != spec['components']:
            raise ValueError(f'Unexpected component count for {source}')
        mj_blob, mj_records, mj_groups = read_parts(mj_path)
        if hashlib.sha256(mj_blob).hexdigest() != spec['mjcf_sha256']:
            raise ValueError(f'MJCF source mesh changed: {source}')
        if len(mj_groups) != len(groups):
            raise ValueError(f'URDF/MJCF component counts differ: {source}')
        matched_groups = []
        for a in groups:
            va = records['vertex'][a]
            matches = [b for b in mj_groups if len(a) == len(b) and np.allclose(
                [va.min((0, 1)), va.max((0, 1))],
                [mj_records['vertex'][b].min((0, 1)), mj_records['vertex'][b].max((0, 1))],
                atol=1e-7, rtol=0)]
            if len(matches) != 1:
                raise ValueError(f'URDF/MJCF component correspondence changed: {source}')
            matched_groups.append(matches[0])
        if len({int(g[0]) for g in matched_groups}) != len(groups):
            raise ValueError(f'URDF/MJCF component mapping is not one-to-one: {source}')
        mj_groups = matched_groups
        component_ids = [i for group in spec['parts'].values() for i in group]
        if sorted(component_ids) != list(range(len(groups))):
            raise ValueError(f'Parts are not a disjoint, complete partition: {source}')
        source_report = {'sha256': spec['sha256'], 'mjcf_sha256': spec['mjcf_sha256'], 'triangles': len(records), 'link': spec['link'], 'parts': {}}
        for name, group_ids in spec['parts'].items():
            ids = np.sort(np.concatenate([groups[i] for i in group_ids]))
            for folder, source_blob, source_records, source_groups in [
                (URDF / visual_dir(spec), blob, records, groups),
                (MJCF / 'assets/visual', mj_blob, mj_records, mj_groups),
            ]:
                source_ids = np.sort(np.concatenate([source_groups[i] for i in group_ids]))
                output = source_blob[:80] + len(source_ids).to_bytes(4, 'little') + source_records[source_ids].tobytes()
                folder.mkdir(parents=True, exist_ok=True)
                (folder / f'{name}.STL').write_bytes(output)
            source_report['parts'][name] = {'triangles': len(ids), 'components': group_ids}
        report['sources'][f'{source}.STL'] = source_report
    (ROOT / 'reports/appearance/motor_visual_partitions.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')


def update_urdf():
    for path in sorted(URDF.glob('par6_*.urdf')):
        text = path.read_text(encoding='utf-8')
        variant = path.stem.removeprefix('par6_')
        for source, spec in variant_specs(variant).items():
            if f'name="{source}_structure_visual"' in text:
                continue
            matches = [m for m in re.finditer(r'(?P<indent>^[ \t]*)<visual>.*?</visual>', text, re.M | re.S)
                       if f'filename="{source_paths(source, spec)[0].relative_to(URDF).as_posix()}"' in m.group(0)]
            if len(matches) != 1:
                raise ValueError(f'Expected one visual for {source}: {path}')
            match = matches[0]
            indent = match.group('indent')
            original = match.group(0)
            visuals = []
            part_materials = materials(spec)
            names = [f'{source}_structure_visual'] + list(part_materials)
            for name in names:
                visual = original.replace('<visual>', f'<visual name="{name}">', 1)
                visual = visual.replace(source_paths(source, spec)[0].relative_to(URDF).as_posix(), f'{visual_dir(spec)}/{name}.STL')
                if name in part_materials:
                    material = part_materials[name]
                    visual = re.sub(r'<material\b.*?</material>',
                                    f'<material name="{material}">\n{indent}    <color rgba="{BLACK}" />\n{indent}  </material>',
                                    visual, count=1, flags=re.S)
                visuals.append(visual)
            text = text[:match.start()] + '\n'.join(visuals) + text[match.end():]
        ET.fromstring(text)
        path.write_text(text, encoding='utf-8', newline='\n')


def update_mjcf():
    for path in sorted(MJCF.glob('par6_*.xml')):
        text = path.read_text(encoding='utf-8')
        variant = path.stem.removeprefix('par6_')
        for source, spec in variant_specs(variant).items():
            if f'name="{source}_structure_visual"' in text:
                continue
            part_materials = materials(spec)
            assets = ['    <!-- Visual-only partitions; original full meshes remain collision assets. -->']
            for material in part_materials.values():
                assets.append(f'    <material name="{material}" rgba="{BLACK}" specular="0.3" shininess="0.25"/>')
            for name in spec['parts']:
                assets.append(f'    <mesh name="{name}" file="visual/{name}.STL" scale="1 1 1"/>')
            text = text.replace('  </asset>', '\n'.join(assets) + '\n  </asset>', 1)
            pattern = r'(?P<indent>^[ \t]*)<geom\b[^>]*mesh="' + spec.get('mjcf_mesh_name', spec['link']) + r'"[^>]*/>'
            matches = list(re.finditer(pattern, text, re.M))
            if len(matches) != 1:
                raise ValueError(f'Expected one full geom for {source}: {path}')
            match = matches[0]
            indent = match.group('indent')
            # Only render metadata changes on the original collision geom.
            collision = re.sub(r'rgba="[^"]*"', 'rgba="0 0 0 0"', match.group(0))
            collision = collision[:-2] + ' group="3"/>'
            lines = [f'{indent}<!-- Original collision geometry and inertia are preserved. -->', collision]
            name = f'{source}_structure_visual'
            lines.append(f'{indent}<geom name="{name}" type="mesh" mesh="{name}" rgba="0.79216 0.81961 0.93333 1" contype="0" conaffinity="0" mass="0" group="2"/>')
            for name, material in part_materials.items():
                lines.append(f'{indent}<geom name="{name}" type="mesh" mesh="{name}" material="{material}" contype="0" conaffinity="0" mass="0" group="2"/>')
            text = text[:match.start()] + '\n'.join(lines) + text[match.end():]
        ET.fromstring(text)
        path.write_text(text, encoding='utf-8', newline='\n')


if __name__ == '__main__':
    generate_meshes()
    update_urdf()
    update_mjcf()
    print('Updated actuator visuals in all variants and lens/motor visuals in the MSG variant.')
