"""Verify exact visual partitions, kinematics and inertias against saved packages.

Finger contacts are intentionally excluded: verify_gripper_contacts.py checks
the later convex decomposition. Contact forces/trajectories need not match.

Example:
  python verify_motor_visuals.py --baseline-urdf PATH --baseline-mujoco PATH
Dependencies: numpy, scipy, mujoco (tested with 3.3.0), shapely, trimesh.
"""

from collections import Counter
from pathlib import Path
import argparse
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from split_motor_visuals import ROOT, URDF, MJCF, SPLITS, read_parts, source_paths, visual_dir, variant_specs, materials
from build_side_badge import BADGE_NAMES, BADGE_MATERIALS, BADGE_TEXTURES

PART_NAMES = {n for spec in SPLITS.values() for n in spec['parts']} | BADGE_NAMES


def current_entry(variant):
    # Appearance equivalence is checked against the original SSG48 finger
    # option. The new default pinch geometry has its own jaw validation.
    return 'ssg48_stock' if variant == 'ssg48' and (MJCF/'par6_ssg48_stock.xml').exists() else variant


def tree_key(element):
    return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
            [tree_key(child) for child in element])


def check_partitions():
    result = {}
    for package, index in [('urdf', 0), ('mujoco', 1)]:
        for source, spec in SPLITS.items():
            _, records, _ = read_parts(source_paths(source, spec)[index])
            folder = URDF / visual_dir(spec) if package == 'urdf' else MJCF / 'assets/visual'
            original = Counter(r.tobytes() for r in records)
            rebuilt = Counter()
            for name in spec['parts']:
                _, part, _ = read_parts(folder / f'{name}.STL')
                rebuilt.update(r.tobytes() for r in part)
            assert rebuilt == original, (package, source, 'Triangle records changed')
            result[f'{package}/{source}'] = len(records)
    return result


def check_urdf(baseline):
    for path in sorted(baseline.rglob('*.STL')):
        assert path.read_bytes() == (URDF / path.relative_to(baseline)).read_bytes(), path
    for variant in ['flange', 'msg', 'ssg48']:
        original = ET.parse(baseline / f'par6_{variant}.urdf').getroot()
        current = ET.parse(URDF / f'par6_{current_entry(variant)}.urdf').getroot()
        affected_links = {s['link'] for s in variant_specs(variant).values()}
        for root in [original, current]:
            for side in ['left','right']:
                link=root.find(f'link[@name="{side}_finger_link"]')
                if link is not None:
                    for collision in list(link.findall('collision')):link.remove(collision)
            for link in root.findall('link'):
                if link.get('name') in affected_links:
                    for visual in link.findall('visual'):
                        link.remove(visual)
        assert tree_key(original) == tree_key(current), (variant, 'Nonvisual URDF changed')
        model = ET.parse(URDF / f'par6_{current_entry(variant)}.urdf').getroot()
        for element in model.findall('.//mesh'):
            assert (URDF / element.get('filename')).is_file(), element.attrib
        expected_parts = {name for spec in variant_specs(variant).values() for name in spec['parts']} | BADGE_NAMES
        for part in expected_parts:
            assert len(model.findall(f'.//visual[@name="{part}"]')) == 1, part


def check_mjcf_xml(baseline, variant):
    original = ET.parse(baseline / f'par6_{variant}.xml').getroot()
    current = ET.parse(MJCF / f'par6_{current_entry(variant)}.xml').getroot()
    for parent in current.iter():
        for child in list(parent):
            name = child.get('name', '')
            material_names = {m for spec in SPLITS.values() for m in materials(spec).values()} | BADGE_MATERIALS
            if name.startswith('contact_') or name in PART_NAMES or (child.tag == 'material' and name in material_names) or name in BADGE_TEXTURES:
                parent.remove(child)
    for side in ['left','right']:
        old_geom=original.find(f'.//body[@name="{side}_finger_link"]/geom')
        new_geom=current.find(f'.//body[@name="{side}_finger_link"]/geom')
        if old_geom is not None:
            new_geom.attrib.clear();new_geom.attrib.update(old_geom.attrib)
    for spec in variant_specs(variant).values():
        mesh_name = spec.get('mjcf_mesh_name', spec['link'])
        old_geom = original.find(f'.//geom[@mesh="{mesh_name}"]')
        new_geom = current.find(f'.//geom[@mesh="{mesh_name}"]')
        new_geom.attrib.clear()
        new_geom.attrib.update(old_geom.attrib)
    assert tree_key(original) == tree_key(current), (variant, 'Nonvisual MJCF changed')


def check_mujoco(baseline):
    for path in sorted((baseline / 'assets').glob('*.STL')):
        assert path.read_bytes() == (MJCF / 'assets' / path.name).read_bytes(), path
    summary = {}
    for variant in ['flange', 'msg', 'ssg48']:
        check_mjcf_xml(baseline, variant)
        old = mujoco.MjModel.from_xml_path(str(baseline / f'scene_{variant}.xml'))
        new = mujoco.MjModel.from_xml_path(str(MJCF / f'scene_{current_entry(variant)}.xml'))
        for name in ['nbody', 'njnt', 'nq', 'nv', 'nu', 'neq', 'ncam', 'nsensor', 'nkey']:
            assert getattr(old, name) == getattr(new, name), (variant, name)
        for prefix in ['jnt_', 'dof_', 'actuator_', 'eq_', 'cam_', 'key_']:
            for name in dir(old):
                if name.startswith(prefix) and isinstance(getattr(old, name), np.ndarray):
                    np.testing.assert_array_equal(getattr(old, name), getattr(new, name), err_msg=f'{variant}/{name}')
        for name in ['body_parentid', 'body_pos', 'body_quat', 'body_ipos', 'body_iquat', 'body_mass', 'body_inertia', 'body_subtreemass', 'body_invweight0']:
            np.testing.assert_array_equal(getattr(old, name), getattr(new, name), err_msg=f'{variant}/{name}')
        new_ids = np.array([i for i in range(new.ngeom) if new.geom(i).name not in PART_NAMES and not new.geom(i).name.startswith('contact_')])
        assert len(new_ids) == old.ngeom
        for name in ['geom_type', 'geom_bodyid', 'geom_dataid', 'geom_pos', 'geom_quat', 'geom_size', 'geom_friction', 'geom_condim', 'geom_solref', 'geom_solimp', 'geom_margin', 'geom_gap', 'geom_priority', 'geom_solmix', 'geom_rbound', 'geom_aabb']:
            np.testing.assert_array_equal(getattr(old, name), getattr(new, name)[new_ids], err_msg=f'{variant}/{name}')
        for i in set(range(new.ngeom)) - set(new_ids):
            if not new.geom(i).name.startswith('contact_'):
                assert new.geom_contype[i] == new.geom_conaffinity[i] == 0
        a, b = mujoco.MjData(old), mujoco.MjData(new)
        rng = np.random.default_rng(27)
        samples = [old.key_qpos[0].copy(), old.key_qpos[1].copy()]
        for _ in range(24):
            q = old.key_qpos[0].copy()
            q[:6] = rng.uniform(-1.5, 1.5, 6)
            samples.append(q)
        for q in samples:
            v = rng.uniform(-.2, .2, old.nv)
            for model, data in [(old, a), (new, b)]:
                mujoco.mj_resetData(model, data)
                data.qpos[:] = q
                data.qvel[:] = v
                data.ctrl[:min(model.nu, len(q))] = q[:model.nu]
                mujoco.mj_forward(model, data)
            for field in ['xpos', 'xmat', 'xipos', 'qM', 'qfrc_bias', 'qfrc_passive', 'qfrc_actuator']:
                np.testing.assert_allclose(getattr(a, field), getattr(b, field), atol=1e-12, rtol=1e-12, err_msg=f'{variant}/{field}')
        summary[variant] = {'current_entry': current_entry(variant),
                            'poses_checked': len(samples),
                            'kinematics_and_mass_properties_unchanged':True,
                            'finger_contact_comparison':'Excluded; intentional later contact refinement.'}
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-urdf', type=Path, required=True)
    parser.add_argument('--baseline-mujoco', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = {'mujoco_version': mujoco.__version__, 'triangle_records_preserved': check_partitions()}
    check_urdf(args.baseline_urdf)
    report['urdf_nonvisual_definitions_except_finger_contacts_unchanged'] = True
    report['mujoco'] = check_mujoco(args.baseline_mujoco)
    if args.output:
        args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
