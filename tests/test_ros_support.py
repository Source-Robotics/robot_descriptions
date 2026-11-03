import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from model_reference import ROOT
from ros_support.robot_description import load_description, select_model

CASES = [(family, tool, jaws, profile) for family in ['par6', 'parol6'] for tool, jaws, profile in [
    ('flange', 'default', 'flange'), ('msg', 'solid', 'msg'),
    ('msg', 'soft_tpu', 'msg_tpu_glue'), ('ssg48', 'solid', 'ssg48'),
    ('ssg48', 'fin_ray', 'ssg48_stock'), ('pneumatic', 'default', 'pneumatic')]]
CASES += [('msg', 'msg', 'solid', 'solid'), ('msg', 'msg', 'soft_tpu', 'soft_tpu'),
          ('ssg48', 'msg', 'solid', 'solid'), ('ssg48', 'msg', 'fin_ray', 'fin_ray')]


@pytest.mark.parametrize('family,tool,jaws,profile', CASES)
def test_ros_selection_and_in_memory_asset_resolution_preserve_each_model(family, tool, jaws, profile):
    filename = select_model(family, tool, jaws)
    assert filename == f'{family}_{profile}.urdf'
    package = ROOT/f'urdf/{family}_description'; path = package/filename
    before = path.read_bytes()
    original = ET.fromstring(before)
    resolved = ET.fromstring(load_description(path, package.name))
    resources = resolved.findall('.//mesh')
    assert resources
    for old, new in zip(original.findall('.//mesh'), resources):
        assert new.get('filename') == f'package://{package.name}/'+old.get('filename')
        assert (package/new.get('filename').split('/', 3)[-1]).is_file()
        new.set('filename', old.get('filename'))
    # Full XML equality checks joints, limits, inertials, visuals, collisions,
    # mimic relations, camera frames and every other model definition.
    assert ET.tostring(original) == ET.tostring(resolved)
    assert path.read_bytes() == before


@pytest.mark.parametrize('family', ['par6','parol6'])
def test_legacy_aliases_and_default_jaws_remain_compatible(family):
    assert select_model(family) == f'{family}_msg.urdf'
    assert select_model(family, 'msg_tpu_glue') == f'{family}_msg_tpu_glue.urdf'
    assert select_model(family, 'ssg48_stock') == f'{family}_ssg48_stock.urdf'


@pytest.mark.parametrize('family,tool,jaws', [
    ('par6','msg','fin_ray'), ('parol6','ssg48','soft_tpu'),
    ('par6','flange','solid'), ('parol6','pneumatic','solid'),
    ('par6','msg_tpu_glue','solid'), ('parol6','ssg48_stock','solid'),
    ('msg','msg','fin_ray'), ('ssg48','msg','soft_tpu'),
    ('other','msg','solid'), ('par6','../../other','default')])
def test_invalid_and_conflicting_selections_fail_clearly(family, tool, jaws):
    with pytest.raises(ValueError):
        select_model(family, tool, jaws)


@pytest.mark.parametrize('filename,error', [('../outside.STL',ValueError), ('missing.STL',FileNotFoundError)])
def test_missing_or_escaping_assets_fail_without_writing_models(tmp_path, filename, error):
    path = tmp_path/'model.urdf'
    path.write_text(f'<robot name="test"><link name="base"><visual><geometry><mesh filename="{filename}"/></geometry></visual></link></robot>')
    before = path.read_bytes()
    with pytest.raises(error):
        load_description(path, 'test_description')
    assert path.read_bytes() == before
