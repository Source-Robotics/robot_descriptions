import pytest

from model_reference import ROOT
from package_layout import verify


@pytest.mark.parametrize('package',[
    'mujoco/par6','mujoco/parol6','mujoco/msg','mujoco/ssg48',
    'urdf/par6_description','urdf/parol6_description',
    'urdf/msg_description','urdf/ssg48_description'])
def test_runtime_package_documentation_and_catalog_links_are_portable(package):
    verify(ROOT/package)
