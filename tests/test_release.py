import hashlib
import json
import zipfile

import pytest

from prepare_release import PACKAGES,prepare


def release_fixture(root,version='2.12.1'):
    package=root/'urdf/par6_description';package.mkdir(parents=True)
    (package/'package.xml').write_text(f'<package><version>{version}</version></package>')
    (root/'releases').mkdir();(root/f'releases/v{version}.md').write_text('Release notes')
    (root/'dist').mkdir()
    for filename,name in PACKAGES.items():
        with zipfile.ZipFile(root/'dist'/filename,'w') as archive:
            archive.writestr(f'{name}/sim2bot.robot.json',json.dumps({'packageVersion':version}))
    (root/'output/pdf').mkdir(parents=True)
    (root/'output/pdf/robot_kinematics_review.pdf').write_bytes(b'%PDF-1.4\nfixture')


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
    assert len(manifest['assets'])==9
    lines=(tmp_path/'dist/SHA256SUMS').read_text().splitlines()
    assert len(lines)==10
    for line in lines:
        digest,name=line.split('  ')
        assert digest==hashlib.sha256((tmp_path/'dist'/name).read_bytes()).hexdigest()
