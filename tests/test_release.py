import hashlib
import json
import zipfile

import pytest

from prepare_release import PACKAGES,MOVEIT_PACKAGES,prepare


def release_fixture(root,version='2.12.1'):
    package=root/'urdf/par6_description';package.mkdir(parents=True)
    (package/'package.xml').write_text(f'<package><version>{version}</version></package>')
    (root/'docs/releases').mkdir(parents=True);(root/f'docs/releases/v{version}.md').write_text('Release notes')
    (root/'dist').mkdir()
    for filename,name in PACKAGES.items():
        with zipfile.ZipFile(root/'dist'/filename,'w') as archive:
            archive.writestr(f'{name}/sim2bot.robot.json',json.dumps({'packageVersion':version}))
    for filename,name in MOVEIT_PACKAGES.items():
        with zipfile.ZipFile(root/'dist'/filename,'w') as archive:
            archive.writestr(f'{name}/package.xml',f'<package><version>{version}</version></package>')
    (root/'docs/kinematics').mkdir(parents=True)
    (root/'docs/kinematics/robot_kinematics_review.pdf').write_bytes(b'%PDF-1.4\nfixture')


def test_release_rejects_a_tag_for_another_version(tmp_path):
    release_fixture(tmp_path)
    with pytest.raises(ValueError,match='does not match package version'):
        prepare(tmp_path,tag='v2.12.0')
    assert not (tmp_path/'dist/SHA256SUMS').exists()


def test_release_rejects_mixed_package_versions(tmp_path):
    release_fixture(tmp_path)
    with zipfile.ZipFile(tmp_path/'dist/parol6_urdf.zip','w') as archive:
        archive.writestr('parol6_description/sim2bot.robot.json',json.dumps({'packageVersion':'2.12.0'}))
    with pytest.raises(ValueError,match='Archive version mismatch'):
        prepare(tmp_path)


def test_release_checksums_match_every_downloadable_asset(tmp_path):
    release_fixture(tmp_path);manifest=prepare(tmp_path,tag='v2.12.1')
    assert len(manifest['assets'])==13
    lines=(tmp_path/'dist/SHA256SUMS').read_text().splitlines()
    assert len(lines)==14
    for line in lines:
        digest,name=line.split('  ')
        assert digest==hashlib.sha256((tmp_path/'dist'/name).read_bytes()).hexdigest()


def test_release_rejects_a_mismatched_moveit_package(tmp_path):
    release_fixture(tmp_path)
    with zipfile.ZipFile(tmp_path/'dist/par6_moveit_config.zip','w') as archive:
        archive.writestr('par6_moveit_config/package.xml','<package><version>0.0.1</version></package>')
    with pytest.raises(ValueError,match='Archive version mismatch'):
        prepare(tmp_path)


def test_release_workflow_publishes_every_prepared_package():
    """the tag job lists its uploads by name: each ZIP prepare_release checks must be among them"""
    from pathlib import Path
    import re
    workflow = (Path(__file__).resolve().parents[1]/'.github/workflows/models.yml').read_text(encoding='utf-8')
    command = workflow[workflow.index('gh release create'):workflow.index('--repo', workflow.index('gh release create'))]
    uploads = set(re.findall(r'dist/([\w.-]+)', command))
    assert set(PACKAGES) | set(MOVEIT_PACKAGES) | {'robot_kinematics_review.pdf', 'release_manifest.json', 'SHA256SUMS'} == uploads

