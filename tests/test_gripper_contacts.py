import json

import mujoco
import numpy as np
import pytest
import trimesh

from model_reference import ROOT
from grasp_check import PROFILES, run_case, validate_initial_placement
from verify_ssg48_contacts import collision_model, sweep


@pytest.mark.parametrize('robot',['par6','parol6'])
@pytest.mark.parametrize('profile',['ssg48','ssg48_stock'])
@pytest.mark.parametrize('format_name',['mujoco','urdf'])
def test_ssg48_can_traverse_full_stroke_without_internal_collision(robot,profile,format_name):
    # The former URDF rack-spanning hulls penetrated the opposing finger by
    # millimetres near closure. Check the actual exported collision meshes.
    result=sweep(collision_model(robot,profile,format_name))
    assert result['passed'],result


@pytest.mark.parametrize('robot',['par6','parol6'])
@pytest.mark.parametrize('profile',PROFILES)
@pytest.mark.parametrize('shape',['box','cylinder'])
def test_grasp_closes_lifts_holds_and_releases_without_internal_interference(robot,profile,shape):
    result=run_case(profile,shape,robot=robot)
    assert result['passed'],result


def test_grasp_rejects_the_old_oversized_ssg48_object():
    path=ROOT/'mujoco/par6/bench/ssg48-solid_box.xml'
    model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model)
    model.geom('object_surface').size[2]=.030
    top=model.geom('bench_table').pos[2]+model.geom('bench_table').size[2]
    data.joint('object_free').qpos[2]=top+.030
    with pytest.raises(ValueError,match='Invalid initial object placement'):
        validate_initial_placement(model,data)


def test_grasp_object_clearance_comes_from_geometry_at_the_same_mass():
    catalog=json.loads((ROOT/'mujoco/par6/bench/profiles.json').read_text())
    for profile,entry in catalog.items():
        assert entry['object_mass_kg']==.05
        assert entry['initial_housing_clearance_m']>=.005-1e-12
        assert entry['initial_table_gap_m']==.0001
        if profile=='ssg48-solid':
            assert entry['object_height_m']==pytest.approx(.0412,abs=1e-6)
        else:
            assert entry['object_height_m']==.060


@pytest.mark.parametrize('robot',['par6','parol6'])
@pytest.mark.parametrize('identifier',['ssg48_solid','ssg48_fin_ray'])
@pytest.mark.parametrize('side',['left','right'])
def test_ssg48_urdf_and_native_share_the_external_contact_envelope(robot,identifier,side):
    for piece in range(6):
        filename=f'{side}_{piece:02d}.STL'
        native=trimesh.load(ROOT/f'mujoco/{robot}/assets/contacts/{identifier}/{filename}',force='mesh')
        portable=trimesh.load(ROOT/f'urdf/{robot}_description/meshes/contacts/{identifier}/{filename}',force='mesh')
        def sorted_vertices(mesh):return mesh.vertices[np.lexsort(mesh.vertices.T)]
        np.testing.assert_array_equal(sorted_vertices(native),sorted_vertices(portable))
