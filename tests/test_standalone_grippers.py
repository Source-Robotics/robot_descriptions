"""Regressions for an isolated, usable gripper rather than an arm or test fixture."""
import shutil
import pytest

from build_standalone_grippers import GRIPPERS
from model_reference import ROOT
from verify_standalone_grippers import check_package, check_profile


@pytest.mark.parametrize('family,suffix,profile',[
    (family,suffix,profile) for family,config in GRIPPERS.items()
    for suffix,profile,_ in config['profiles']])
def test_gripper_preserves_mounted_frames_physics_and_actual_close_open(family,suffix,profile):
    check_profile(family,suffix,profile)


@pytest.mark.parametrize('family',['msg','ssg48'])
@pytest.mark.parametrize('native',[True,False])
def test_gripper_packages_work_after_relocation_without_robot_assets(tmp_path,family,native):
    path = f'mujoco/{family}' if native else f'urdf/{family}_description'
    source = ROOT/path
    relocated = tmp_path/source.name
    shutil.copytree(source,relocated)
    check_package(relocated,native)
