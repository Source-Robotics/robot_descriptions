import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from model_reference import ROOT
from sync_model_dynamics import body_properties,write_native


@pytest.mark.parametrize('profile,body,fixed,total',[
    ('flange','flange_link',.0555,.0555),('msg','msg_gripper_link',.3014,.380),
    ('msg_tpu_glue','msg_gripper_link',.3014,.380),('ssg48','ssg48_gripper_link',.3614,.440),
    ('ssg48_stock','ssg48_gripper_link',.3614,.440)])
def test_standalone_variants_use_measured_component_masses(profile,body,fixed,total):
    model=mujoco.MjModel.from_xml_path(str(ROOT/f'mujoco/par6/scene_{profile}.xml'))
    urdf=ET.parse(ROOT/f'urdf/par6_description/par6_{profile}.urdf')
    names=[body] if profile=='flange' else [body,'left_finger_link','right_finger_link']
    expected=[fixed] if profile=='flange' else [fixed,.0393,.0393]
    native=[model.body(name).mass[0] for name in names]
    portable=[float(urdf.find(f'link[@name="{name}"]/inertial/mass').get('value')) for name in names]
    np.testing.assert_allclose(native,expected,rtol=0,atol=1e-14)
    np.testing.assert_allclose(portable,expected,rtol=0,atol=1e-14)
    assert sum(native)==pytest.approx(total,abs=1e-14)


def test_coordinate_conversion_and_native_encoding_preserve_off_diagonal_inertia():
    reference={'com_m':[1.,2.,3.],'inertia_kg_m2':[[.12,.01,.02],[.01,.18,.03],[.02,.03,.20]]}
    body=np.array([[0.,-1,0,0],[1,0,0,1],[0,0,1,0],[0,0,0,1]])
    com,tensor,_=body_properties(reference,np.eye(4),body)
    expected=np.array([[.18,-.01,.03],[-.01,.12,-.02],[.03,-.02,.20]])
    np.testing.assert_allclose(com,[1.,-1.,3.]);np.testing.assert_allclose(tensor,expected)
    inertial=ET.Element('inertial');write_native(inertial,2.,com,tensor)
    model=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="test"><freejoint/>'+ET.tostring(inertial,encoding='unicode')+'</body></worldbody></mujoco>')
    rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,model.body('test').iquat)
    R=rotation.reshape(3,3)
    np.testing.assert_allclose(R@np.diag(model.body('test').inertia)@R.T,expected,atol=1e-15)
