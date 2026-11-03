import json

import mujoco
import numpy as np
import pytest

from build_review_sheets import (ROBOTS, PROFILES, CACHE, paths, dimensions,
                                 dh_frames, urdf_fk, ARM_JOINTS, error,
                                 cached_snapshot, project, visual_signature)


@pytest.mark.parametrize('robot', ROBOTS)
@pytest.mark.parametrize('profile', PROFILES)
def test_tool_specific_six_row_dh_matches_actual_urdf_tcp(robot, profile):
    """Held-out angles catch sign, theta-offset and fixed-tool errors."""
    upath, _ = paths(robot, profile)
    dims, tool = dimensions(upath)
    for q in ([.41, -.37, .29, .63, -.52, .87], [-.22, .31, -.71, -.39, .64, -1.13]):
        tcp = urdf_fk(upath, dict(zip(ARM_JOINTS, q)))['tcp']
        residual = error(dh_frames(q, dims)[-1], tcp)
        assert residual[0] < .01 and residual[1] < .002
    assert dims[4] == pytest.approx(.177)
    reach = .0571 if profile == 'flange' else .0689 if profile == 'pneumatic' else .1409
    assert dims[5] == pytest.approx(reach if robot == 'par6' else reach-.0066)
    assert dims[6] == pytest.approx(.02835 if profile.startswith('msg') else .04525 if profile == 'pneumatic' else 0.)
    np.testing.assert_allclose(tool[:3, :3], np.eye(3), atol=1e-12)


def test_render_cache_tracks_geometry_and_projection_is_registered(monkeypatch):
    model = mujoco.MjModel.from_xml_path(str(paths('par6', 'msg')[1]))
    _, camera = cached_snapshot(model, 'par6', 'msg')
    assert camera['orthographic']
    assert camera['registration_error_max_px'] < 1.5
    base = project([0., 0., 0.], camera)
    moved_x, moved_z = project([.1, 0., 0.], camera), project([0., 0., .1], camera)
    assert moved_x[0] < base[0] and moved_z[1] < base[1]
    assert abs(moved_x[0]-base[0]) == pytest.approx(abs(moved_z[1]-base[1]), abs=1e-6)
    monkeypatch.setattr('build_review_sheets.visual_signature', lambda path, **kwargs: 'changed-source')
    with pytest.raises(ValueError, match='stale review snapshot'):
        cached_snapshot(model, 'par6', 'msg')


def test_render_signature_tracks_geometry_assets_and_ignores_inertia(tmp_path):
    scene = tmp_path/'scene.xml'
    mesh = tmp_path/'part.STL'
    mesh.write_bytes(b'geometry version one')
    def definition(x, mass):
        scene.write_text(f'<mujoco><asset><mesh file="part.STL"/></asset><worldbody><body pos="{x} 0 0"><inertial mass="{mass}"/></body></worldbody></mujoco>')
    definition(0, 1)
    first = visual_signature(scene)
    definition(0, 2)
    assert visual_signature(scene) == first
    definition(1, 2)
    assert visual_signature(scene) != first
    definition(0, 1)
    mesh.write_bytes(b'geometry version two')
    assert visual_signature(scene) != first


def test_all_profiles_share_one_camera_scale_and_have_passing_audits():
    from parol6_model import ROOT
    audits = json.loads((ROOT/'reports/kinematics/review/validation.json').read_text())
    assert len(audits) == len(ROBOTS)*len(PROFILES)
    assert {(d['robot'], d['profile']) for d in audits} == {(r, p) for r in ROBOTS for p in PROFILES}
    for item in audits:
        assert item['poses_checked'] == 128
        assert item['camera'] == audits[0]['camera']
        for value in item['max_error_mm_deg'].values():
            assert value[0] < .025 and value[1] < .01
        assert item['snapshot_registration_error_max_px'] < 1.5
