import json
import xml.etree.ElementTree as ET

import pytest

from model_reference import ROOT


@pytest.mark.parametrize('package',[
    'mujoco/par6','mujoco/parol6','mujoco/msg','mujoco/ssg48',
    'urdf/par6_description','urdf/parol6_description',
    'urdf/msg_description','urdf/ssg48_description'])
def test_downloaded_model_license_and_support_code_scope_are_consistent(package):
    path=ROOT/package
    data=json.loads((path/'sim2bot.robot.json').read_text())
    assert data['license']=='CC-BY-4.0'
    assert (path/'LICENSE').read_bytes()==(ROOT/'LICENSE').read_bytes()
    assert 'Source Robotics' in (path/'docs/ATTRIBUTION.md').read_text()
    if (path/'package.xml').exists():
        assert data['codeLicense']=='Apache-2.0'
        assert {p.text for p in ET.parse(path/'package.xml').findall('license')}=={'CC-BY-4.0','Apache-2.0'}
        assert (path/'docs/LICENSE-CODE.txt').read_bytes()==(ROOT/'LICENSES/Apache-2.0.txt').read_bytes()
