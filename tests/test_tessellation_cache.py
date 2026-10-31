import copy
import json

import gmsh
import pytest

import build_msg_jaws as jaws


def test_checked_step_tessellation_is_reused_without_platform_remeshing(monkeypatch):
    config=json.loads(jaws.CONFIG.read_text(encoding='utf-8'))
    def unexpected():raise AssertionError('Unchanged STEP should use its checked tessellation')
    monkeypatch.setattr(gmsh,'initialize',unexpected)
    meshes,report,version=jaws.step_solids(config)
    assert set(meshes)=={'holder','hex'}
    assert all(mesh.is_volume for mesh in meshes.values())
    assert all(len(meshes[name].faces)==entry['triangles'] for name,entry in report.items())
    assert version==gmsh.__version__


def test_changed_meshing_settings_do_not_reuse_the_checked_tessellation(monkeypatch):
    config=copy.deepcopy(json.loads(jaws.CONFIG.read_text(encoding='utf-8')))
    config['mesh_size_max_mm']+=1
    def remesh():raise RuntimeError('remesh requested')
    monkeypatch.setattr(gmsh,'initialize',remesh)
    with pytest.raises(RuntimeError,match='remesh requested'):
        jaws.step_solids(config)
