"""Non-default themes must propagate without changing the simulated mechanism."""
import copy
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from model_colors import ROOT, apply_tree, badge_config, color, load, rgba, validate

MODELS = sorted((ROOT/'mujoco').glob('*/*.xml'))
MODELS = [p for p in MODELS if p.stem.startswith(p.parent.name+'_') or p.stem == p.parent.name]
MODELS += sorted((ROOT/'urdf').glob('*/*.urdf'))


def themed_palette():
    palette = copy.deepcopy(load())
    palette['colors'].update(structure='#208040', motors='#2060E0', camera_lens='#8040C0',
                             msg_solid_jaws='#C04080', msg_soft_holders='#106080',
                             msg_soft_inserts='#40C0A0', ssg48_solid_jaws='#E08020',
                             ssg48_fin_ray_jaws='#6040A0', pneumatic_body='#80A020',
                             pneumatic_jaws='#A02040')
    palette['overrides'] = {'j1_motor': '#F08020', 'parol6_structure': '#4080C0'}
    return palette


def profile_for(path):
    family = path.parent.name.removesuffix('_description')
    profile = path.stem.removeprefix(family+'_')
    if family in {'msg', 'ssg48'}:
        profile = {'solid': 'msg' if family == 'msg' else 'ssg48',
                   'soft_tpu': 'msg_tpu_glue', 'fin_ray': 'ssg48_stock'}[profile]
    return family, profile


def compile_tree(root, directory):
    root = copy.deepcopy(root)
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler')
    for key in ['meshdir', 'texturedir']:
        compiler.set(key, str(directory/compiler.get(key, '')))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


@pytest.mark.parametrize('path', MODELS, ids=lambda p: f'{p.parent.name}/{p.name}')
def test_non_default_theme_changes_visuals_and_preserves_all_physics(path):
    original = ET.parse(path).getroot()
    themed = copy.deepcopy(original)
    palette = themed_palette()
    family, profile = profile_for(path)
    apply_tree(themed, family, profile, palette)
    assert ET.tostring(themed) != ET.tostring(original)
    if path.suffix == '.urdf':
        # Removing complete visual blocks leaves collision, inertia, joints,
        # limits, sensors and every other definition exactly as supplied.
        for tree in [original, themed]:
            for link in tree.findall('link'):
                for visual in link.findall('visual'):
                    link.remove(visual)
        assert ET.tostring(original) == ET.tostring(themed)
        return
    previous = compile_tree(original, path.parent)
    selected = compile_tree(themed, path.parent)
    changed = set()
    for name in dir(previous):
        before, after = getattr(previous, name), getattr(selected, name)
        if isinstance(before, np.ndarray):
            # a degenerate (zero-area) CAD polygon has a NaN normal in both compiles
            if not np.array_equal(before, after, equal_nan=np.issubdtype(before.dtype, np.floating)):
                changed.add(name)
    # This compares every compiled array, including all mesh vertices/faces,
    # contact parameters, body inertials, joint/actuator data and camera poses.
    assert changed and changed <= {'geom_rgba', 'mat_rgba'}, changed
    if family in {'par6', 'parol6'}:
        np.testing.assert_allclose(selected.mat_rgba[selected.geom('j1_actuator_visual').matid[0]],
                                   rgba('#F08020'), atol=3e-8, rtol=0)
        np.testing.assert_allclose(selected.mat_rgba[selected.geom('j2_actuator_visual').matid[0]],
                                   rgba('#2060E0'), atol=3e-8, rtol=0)
    if profile.startswith('msg'):
        np.testing.assert_allclose(selected.mat_rgba[selected.geom('msg_camera_lens_visual').matid[0]],
                                   rgba('#8040C0'), atol=3e-8, rtol=0)
    # Reapplying a palette does not introduce more changes.
    once = ET.tostring(themed)
    apply_tree(themed, family, profile, palette)
    assert ET.tostring(themed) == once


def test_badge_uses_shared_colors_without_mutating_geometry_settings():
    source = json.loads((ROOT/'source/artwork/par6_side_badge.json').read_text())
    palette = themed_palette()
    palette['badge']['wordmark'] = '#8040C0'
    result = badge_config(source, palette)
    assert result['panel_color'] == '#208040'
    assert result['colors']['wordmark'] == '#8040C0'
    assert source == json.loads((ROOT/'source/artwork/par6_side_badge.json').read_text())
    assert {k: v for k, v in result.items() if k not in {'colors', 'panel_color'}} == {
        k: v for k, v in source.items() if k not in {'colors', 'panel_color'}}
    palette['overrides']['par6_structure'] = '#123456'
    assert badge_config(source, palette)['panel_color'] == '#123456'
    palette['overrides']['par6_panel'] = '#654321'
    assert badge_config(source, palette)['panel_color'] == '#654321'
    assert color(palette, 'j1_motor') == '#F08020'
    assert color(palette, 'msg_motor') == '#2060E0'


@pytest.mark.parametrize('setting,value', [
    ('motors', '#GG1234'), ('structure', [1, 0, 0]),
    ('camera_lens', [0, 0, float('nan'), 1]), ('motors', [True, 0, 0, 1]),
    ('structure', [2, 0, 0, 1]), ('motors', [0, 0, 0, 0]),
])
def test_invalid_palette_fails_before_models_are_written(setting, value):
    palette = themed_palette(); palette['colors'][setting] = value
    with pytest.raises(ValueError, match=setting):
        validate(palette)


def test_palette_rejects_typos_and_duplicate_keys(tmp_path):
    palette = themed_palette(); palette['overrides']['j1_motr'] = '#123456'
    with pytest.raises(ValueError, match='overrides'):
        validate(palette)
    target = tmp_path/'palette.json'
    target.write_text('{"schema_version":1,"schema_version":2}')
    with pytest.raises(ValueError, match='Duplicate'):
        load(target)
