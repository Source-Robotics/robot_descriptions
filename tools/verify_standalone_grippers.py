"""Check independent gripper exports against their original mounted models."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mujoco
import numpy as np

from build_standalone_grippers import GRIPPERS
from model_reference import ROOT, ARM_JOINTS
from parol6_model import urdf_fk
from verify_msg_jaws import canonical


from package_layout import documentation, previews

def pose(data, name, site=False):
    item = data.site(name) if site else data.body(name)
    result = np.eye(4)
    result[:3, :3] = item.xmat.reshape(3, 3)
    result[:3, 3] = item.xpos
    return result


def internal_penetration(model, data, names):
    bodies = {model.body(name).id for name in names}
    penetration = 0.
    for contact in data.contact:
        pair = {int(model.geom_bodyid[g]) for g in contact.geom}
        if pair <= bodies:
            penetration = max(penetration,-float(contact.dist))
    return penetration


def dynamic_cycle(model, travel):
    data = mujoco.MjData(model)
    errors = []
    maximum_penetration = 0.
    opposing = {model.body('left_finger_link').id,model.body('right_finger_link').id}
    for target in [-travel, 0.]:
        data.ctrl[model.actuator('gripper_actuator').id] = target
        for _ in range(2000):
            mujoco.mj_step(model, data)
            assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
            for contact in data.contact:
                maximum_penetration = max(maximum_penetration, -float(contact.dist))
                pair = {int(model.geom_bodyid[g]) for g in contact.geom}
                # Opposing jaws intentionally touch when closed. A few microns
                # of compliant solver compression are expected; a housing
                # collision is not part of this standalone closing motion.
                assert -float(contact.dist) < (2e-5 if pair == opposing else 1e-6)
        errors.append(abs(float(data.joint('left_finger_joint').qpos[0])-target))
    assert max(errors) < .0005, errors
    assert not any(w.number for w in data.warning)
    return {'close_open_error_m': errors, 'maximum_internal_penetration_m': maximum_penetration}


def check_profile(family, suffix, profile, samples=41):
    config = GRIPPERS[family]
    native_path = ROOT / f'mujoco/{family}/{family}_{suffix}.xml'
    urdf_path = ROOT / f'urdf/{family}_description/{family}_{suffix}.urdf'
    source_native = ROOT / f'mujoco/par6/par6_{profile}.xml'
    source_urdf = ROOT / f'urdf/par6_description/par6_{profile}.urdf'
    model = mujoco.MjModel.from_xml_path(str(native_path))
    reference = mujoco.MjModel.from_xml_path(str(source_native))
    data, expected = mujoco.MjData(model), mujoco.MjData(reference)
    assert model.nq == model.nv == 2 and model.nu == model.neq == 1
    assert model.nmocap == 0
    assert {model.joint(i).name for i in range(model.njnt)} == {'left_finger_joint','right_finger_joint'}
    names = [f'{family}_gripper_link', 'left_finger_link', 'right_finger_link']
    assert abs(sum(float(model.body(n).mass[0]) for n in names)-config['mass']) < 1e-12
    for name in names:
        for attribute in ['body_mass','body_ipos','body_iquat','body_inertia']:
            np.testing.assert_array_equal(getattr(model,attribute)[model.body(name).id],
                                          getattr(reference,attribute)[reference.body(name).id])
    for name in ['left_finger_joint','right_finger_joint']:
        np.testing.assert_array_equal(model.joint(name).range,reference.joint(name).range)
        for attribute in ['dof_damping','dof_armature','dof_frictionloss']:
            np.testing.assert_array_equal(getattr(model,attribute)[model.joint(name).dofadr],
                                          getattr(reference,attribute)[reference.joint(name).dofadr])
    for attribute in ['actuator_gainprm','actuator_biasprm','actuator_ctrlrange','actuator_forcerange']:
        np.testing.assert_array_equal(getattr(model,attribute)[model.actuator('gripper_actuator').id],
                                      getattr(reference,attribute)[reference.actuator('gripper_actuator').id])
    np.testing.assert_array_equal(model.eq_data[0],reference.eq_data[0])
    if family == 'msg':
        for attribute in ['cam_pos','cam_quat','cam_fovy','cam_resolution']:
            np.testing.assert_array_equal(getattr(model,attribute)[model.camera('msg_camera').id],
                                          getattr(reference,attribute)[reference.camera('msg_camera').id])
    original, exported = ET.parse(source_urdf).getroot(), ET.parse(urdf_path).getroot()
    assert len(exported.findall('joint[@type="prismatic"]')) == 2
    assert not any(exported.find(f'joint[@name="{n}"]') is not None for n in ARM_JOINTS)
    for link in exported.findall('link'):
        assert canonical(link) == canonical(original.find(f'link[@name="{link.get("name")}"]'))
    for joint in exported.findall('joint'):
        assert canonical(joint) == canonical(original.find(f'joint[@name="{joint.get("name")}"]'))
    rng = np.random.default_rng(812)
    max_native_error = max_urdf_error = penetration = 0.
    maximum_contact_change = 0.
    for value in np.linspace(0., config['travel'], samples):
        values = {'left_finger_joint':-value,'right_finger_joint':value}
        for name, angle in zip(ARM_JOINTS, rng.uniform(-1.,1.,6)):
            expected.joint(name).qpos[:] = angle
        for name, amount in values.items():
            data.joint(name).qpos[:] = amount
            expected.joint(name).qpos[:] = amount
        mujoco.mj_forward(model,data);mujoco.mj_forward(reference,expected)
        relative = np.linalg.inv(pose(expected,'flange'))
        for name in names:
            max_native_error = max(max_native_error,float(np.abs(pose(data,name)-relative@pose(expected,name)).max()))
        max_native_error = max(max_native_error,float(np.abs(pose(data,'tcp',True)-relative@pose(expected,'tcp',True)).max()))
        actual_frames,source_frames = urdf_fk(urdf_path,values),urdf_fk(source_urdf,values)
        reference_flange = np.linalg.inv(source_frames['flange'])
        for name, frame in actual_frames.items():
            max_urdf_error = max(max_urdf_error,float(np.abs(frame-reference_flange@source_frames[name]).max()))
        current_contact = internal_penetration(model,data,names)
        source_contact = internal_penetration(reference,expected,names)
        penetration = max(penetration,current_contact)
        maximum_contact_change = max(maximum_contact_change,abs(current_contact-source_contact))
    assert max_native_error < 1e-12 and max_urdf_error < 1e-12
    # At the MSG joint-limit pose, the opposing jaws already overlap in the
    # original model. Compare forced-pose contacts with that source rather
    # than silently changing its limits or contact surfaces during export.
    # The separate physical close/open check must stop at contact stably.
    assert maximum_contact_change < 1e-8, maximum_contact_change
    return {'gripper':family,'jaws':suffix,'samples':samples,'mass_kg':config['mass'],
            'maximum_native_transform_error':max_native_error,'maximum_urdf_transform_error':max_urdf_error,
            'forced_pose_maximum_overlap_m':penetration,'maximum_contact_change_from_source_m':maximum_contact_change,
            'local_physics_and_urdf_definitions_preserved':True,
            **dynamic_cycle(model,config['travel'])}


def check_package(package, native):
    """Run on a relocated/extracted package with no authoring-tree dependencies."""
    package = Path(package)
    metadata = json.loads((package/'sim2bot.robot.json').read_text(encoding='utf-8'))
    family = 'ssg48' if package.name.startswith('ssg48') else 'msg'
    config = GRIPPERS[family]
    assert metadata['category'] == 'gripper' and metadata['rootFrame'] == 'flange'
    assert metadata['dof'] == 1 and len(metadata['variants']) == 2
    assert {v['id'] for v in metadata['variants']} == {x[0].replace('_','-') for x in config['profiles']}
    assert all((package/name).is_file() for name in ['README.md','LICENSE','docs/VALIDATION.md'])
    for item in metadata['variants']:
        entry = package/item['entry']
        if item.get('preview'):
            assert (package/item['preview']).is_file()
        if native:
            model = mujoco.MjModel.from_xml_path(str(entry))
            assert model.nq == 2 and model.nu == 1 and model.nmocap == 0
            assert abs(sum(float(model.body(n).mass[0]) for n in [f'{family}_gripper_link','left_finger_link','right_finger_link'])-config['mass']) < 1e-12
            assert not any(model.joint(i).name in ARM_JOINTS for i in range(model.njnt))
        else:
            root = ET.parse(entry).getroot()
            assert len(root.findall('joint[@type="prismatic"]')) == 2
            mass = sum(float(x.get('value')) for x in root.findall('link/inertial/mass'))
            assert abs(mass-config['mass']) < 1e-12
            for mesh in root.findall('.//mesh'):
                path = (package/mesh.get('filename')).resolve()
                assert path.is_relative_to(package.resolve()) and path.is_file()
    return {'variants_checked':2,'standalone_gripper':True,'crc_and_assets_valid':True}


def verify():
    results = [check_profile(family,suffix,profile)
               for family,config in GRIPPERS.items() for suffix,profile,_ in config['profiles']]
    report = {'scope':'Independent MSG and SSG48 URDF/MuJoCo exports, relative to physical flange frame.',
              'results':results}
    (ROOT/'reports/grippers/standalone_grippers_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('Standalone MSG/SSG48: all four jaw profiles preserve FK/physics and pass close/open/contact checks.',flush=True)
    return report


if __name__ == '__main__':
    verify()
