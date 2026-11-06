import json
import pytest

from model_reference import ROOT
from package_layout import verify


@pytest.mark.parametrize('package',[
    'mujoco/par6','mujoco/parol6','mujoco/msg','mujoco/ssg48',
    'urdf/par6_description','urdf/parol6_description',
    'urdf/msg_description','urdf/ssg48_description',
    'mujoco/raft','urdf/raft_description'])
def test_runtime_package_documentation_and_catalog_links_are_portable(package):
    verify(ROOT/package)
    assert json.loads((ROOT/package/'docs/colors.json').read_text()) == json.loads(
        (ROOT/'source/appearance/colors.json').read_text())
