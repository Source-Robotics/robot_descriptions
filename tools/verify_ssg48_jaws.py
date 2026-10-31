"""Check the default pinch fingers against the retained original SSG48 model."""

from collections import Counter
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from split_motor_visuals import ROOT, MJCF, URDF, read_parts
from build_ssg48_jaws import CONFIG, records_for
from verify_msg_jaws import canonical


def verify():
    old=mujoco.MjModel.from_xml_path(str(MJCF/'scene_ssg48_stock.xml'))
    new=mujoco.MjModel.from_xml_path(str(MJCF/'scene_ssg48.xml'))
    assert (old.nbody,old.njnt,old.ngeom,old.nq,old.nv,old.nu)==(new.nbody,new.njnt,new.ngeom,new.nq,new.nv,new.nu)
    arrays=['body_parentid','body_pos','body_quat','body_ipos','body_iquat','body_mass','body_inertia',
            'jnt_type','jnt_pos','jnt_axis','jnt_range','dof_damping','dof_armature','dof_frictionloss',
            'actuator_trnid','actuator_gainprm','actuator_biasprm','actuator_ctrlrange','actuator_forcerange',
            'eq_data','eq_obj1id','eq_obj2id','key_qpos','key_ctrl']
    for name in arrays:
        np.testing.assert_array_equal(getattr(old,name),getattr(new,name),err_msg=name)
    a,b=mujoco.MjData(old),mujoco.MjData(new)
    rng=np.random.default_rng(2710)
    for _ in range(26):
        q=np.r_[rng.uniform(-1,1,6),rng.uniform(-.0235,0),0]
        q[-1]=-q[-2]
        for model,data in [(old,a),(new,b)]:
            data.qpos[:]=q
            mujoco.mj_forward(model,data)
        for name in ['xpos','xmat','qM','qfrc_bias','site_xpos','site_xmat']:
            np.testing.assert_array_equal(getattr(a,name),getattr(b,name),err_msg=name)
    mujoco.mj_resetDataKeyframe(new,b,0)
    b.ctrl[new.actuator('gripper_actuator').id]=-.0235
    for _ in range(1000):
        mujoco.mj_step(new,b)
        assert np.all(np.isfinite(b.qpos)) and np.all(np.isfinite(b.qvel))
    coupling_error=float(abs(b.qpos[new.joint('left_finger_joint').qposadr[0]]+
                              b.qpos[new.joint('right_finger_joint').qposadr[0]]))
    assert coupling_error<1e-4
    before,after=ET.parse(URDF/'par6_ssg48_stock.urdf').getroot(),ET.parse(URDF/'par6_ssg48.urdf').getroot()
    after.set('name',before.get('name'))
    for tree in [before,after]:
        for side in ['left','right']:
            link=tree.find(f'link[@name="{side}_finger_link"]')
            for element in list(link):
                if element.tag in ['visual','collision']:link.remove(element)
    assert canonical(before)==canonical(after),'Non-jaw URDF definitions changed'
    config=json.loads(CONFIG.read_text())
    meshes={}
    for package,original_folder,new_folder in [
        (MJCF,MJCF/'assets',MJCF/'assets/jaws/ssg48_pinch'),
        (URDF,URDF/'meshes/ssg48',URDF/'meshes/ssg48/jaws/pinch'),
    ]:
        for i,side in enumerate(['left','right'],1):
            source=original_folder/(f'jaw{i}_SSG48.STL' if package==MJCF else f'jaw{i}.STL')
            _,carrier=records_for(source,True)
            _,finger=records_for(ROOT/config['matching_export']/f'jaw{i}.STL',False)
            _,combined,_=read_parts(new_folder/f'{side}.STL')
            assert Counter(r.tobytes() for r in combined)==Counter(r.tobytes() for r in np.concatenate([carrier,finger]))
            meshes[(new_folder/f'{side}.STL').relative_to(ROOT).as_posix()]={
                'exact_carriage_and_verified_finger_records':True,'triangles':len(combined)}
    report={'mujoco_version':mujoco.__version__,'default':'pinch','poses_checked':26,
            'unchanged_compiled_arrays':arrays,'arm_and_tcp_kinematics_unchanged':True,
            'urdf_non_jaw_definitions_unchanged':True,'jaw_closing_steps':1000,
            'closing_coupling_error_m':coupling_error,'geoms_before':old.ngeom,'geoms_after':new.ngeom,
            'meshes':meshes,'inertia_scope':'Existing finger mass and inertia retained as an approximation; new collision geometry is intentional.'}
    (ROOT/'tools/ssg48_jaw_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':verify()
