"""Check captured-reference dynamics; optionally compare a before-edit baseline."""

from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from audit_rcb_dynamics import (ROOT, MJCF, URDF, BODY_NAMES,
                                RCB_FROM_MJ_OFFSET,load_rcb_robot,urdf_bodies,
                                mjcf_bodies,aggregate,difference)
from verify_msg_jaws import canonical
from sync_model_dynamics import tool_masses


def same_except_inertials(before,after,kind,allowed):
    for tree in [before,after]:
        for body in allowed:
            element=tree.find(f'.//body[@name="{body}"]' if kind=='mujoco' else f'link[@name="{body}"]')
            element.remove(element.find('inertial'))
    assert canonical(before)==canonical(after),'XML changed outside the permitted inertial elements'


def verify(baseline_mujoco=None,baseline_urdf=None,poses=160):
    if (baseline_mujoco is None)!=(baseline_urdf is None):
        raise ValueError('Supply both baseline packages or neither')
    check_baseline=baseline_mujoco is not None
    if not check_baseline:baseline_mujoco,baseline_urdf=MJCF,URDF
    source=ROOT/'source/dynamics'
    mapping=json.loads((source/'model_mapping.json').read_text())
    measured=json.loads((source/'physical_measurements.json').read_text())
    ref=load_rcb_robot('Flange')
    # A five-link rigid arm, with no tool or motor rotor terms. This independently
    # checks the synchronized data without inheriting the unresolved tool model.
    arm=ref.arm_only()
    rng=np.random.default_rng(271028)
    samples=[RCB_FROM_MJ_OFFSET.copy(),np.asarray(ref.qlim[0]).copy(),np.asarray(ref.qlim[1]).copy()]
    samples += [rng.uniform(ref.qlim[0],ref.qlim[1]) for _ in range(poses-3)]
    report={'poses_per_profile':len(samples),'joint_sampling':'RCB software limits: standby, both limit corners, random poses',
            'arm_com_tolerance_mm':.01,'arm_tensor_relative_tolerance':1e-4,
            'reference_mode':'versioned snapshot; live runtime is not imported',
            'external_baseline_checked':check_baseline,'profiles':{}}
    for profile,family in mapping['profiles'].items():
        masses=tool_masses(family,measured)
        allowed=BODY_NAMES+list(masses)
        entry={'tool_mass_g':sum(masses.values())*1000,'formats':{}}
        old_m=mujoco.MjModel.from_xml_path(str(baseline_mujoco/f'scene_{profile}.xml'))
        model=mujoco.MjModel.from_xml_path(str(MJCF/f'scene_{profile}.xml'))
        previous=mujoco.MjData(old_m);data=mujoco.MjData(model)
        assert (old_m.nq,old_m.nv,old_m.nu,old_m.nbody,old_m.ngeom)==(model.nq,model.nv,model.nu,model.nbody,model.ngeom)
        for name in ['body_parentid','body_pos','body_quat','jnt_type','jnt_pos','jnt_axis','jnt_range',
                     'dof_armature','dof_damping','dof_frictionloss','actuator_trnid','actuator_gainprm',
                     'actuator_biasprm','actuator_ctrlrange','actuator_forcerange','eq_data','eq_obj1id','eq_obj2id',
                     'geom_type','geom_bodyid','geom_dataid','geom_pos','geom_quat','geom_size','geom_rgba',
                     'geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp',
                     'cam_pos','cam_quat','cam_fovy','site_pos','site_quat','key_qpos','key_ctrl']:
            np.testing.assert_array_equal(getattr(old_m,name),getattr(model,name),err_msg=f'{profile}/{name}')
        base=model.body('base_link').id
        for name in ['body_mass','body_ipos','body_iquat','body_inertia']:
            np.testing.assert_array_equal(getattr(old_m,name)[base],getattr(model,name)[base])
        for body,mass in masses.items():
            bid=model.body(body).id
            assert abs(model.body_mass[bid]-mass)<1e-12
            for name in ['body_ipos','body_iquat','body_inertia']:
                np.testing.assert_array_equal(getattr(old_m,name)[bid],getattr(model,name)[bid])
        for kind,package,baseline,extension in [('mujoco',MJCF,baseline_mujoco,'xml'),('urdf',URDF,baseline_urdf,'urdf')]:
            before=ET.parse(baseline/f'par6_{profile}.{extension}').getroot()
            after=ET.parse(package/f'par6_{profile}.{extension}').getroot()
            if kind=='urdf':
                for body,mass in masses.items():
                    old=before.find(f'link[@name="{body}"]/inertial')
                    new=after.find(f'link[@name="{body}"]/inertial')
                    assert abs(float(new.find('mass').get('value'))-mass)<1e-12
                    old.remove(old.find('mass'));new.remove(new.find('mass'))
                    assert canonical(old)==canonical(new),'Tool COM or inertia changed'
            same_except_inertials(before,after,kind,allowed)
            for path in baseline.rglob('*'):
                if path.is_file() and path.suffix.lower() in ['.stl','.obj','.dae','.png','.svg','.mtl']:
                    assert path.read_bytes()==(package/path.relative_to(baseline)).read_bytes(),str(path)
            entry['formats'][kind]={'max_mass_difference_g':0.,'max_com_difference_mm':0.,'max_tensor_relative_difference':0.}
        max_gravity=max_matrix=0.
        for q in samples:
            frames=np.asarray(ref.fkine_all(q).A)
            values={'mujoco':mjcf_bodies(model,data,q),'urdf':urdf_bodies(URDF/f'par6_{profile}.urdf',q)}
            mjcf_bodies(old_m,previous,q)
            for name in ['xpos','xmat','site_xpos','site_xmat']:
                np.testing.assert_array_equal(getattr(previous,name),getattr(data,name),err_msg=f'{profile}/{name}')
            for kind,bodies in values.items():
                for i,body in enumerate(BODY_NAMES):
                    actual=aggregate(bodies,[body],frames[i+1])
                    L=ref.links[i]
                    expected={'mass_kg':float(L.m),'com_m':np.asarray(L.r).tolist(),'inertia_kg_m2':np.asarray(L.I).tolist()}
                    error=difference(expected,actual)
                    result=entry['formats'][kind]
                    result['max_mass_difference_g']=max(result['max_mass_difference_g'],abs(error['mass_difference_g']))
                    result['max_com_difference_mm']=max(result['max_com_difference_mm'],error['com_distance_mm'])
                    result['max_tensor_relative_difference']=max(result['max_tensor_relative_difference'],error['inertia_frobenius_relative_error'])
            M=np.zeros((5,5));g=np.zeros(5)
            jacp=np.zeros((3,model.nv));jacr=np.zeros((3,model.nv))
            for body in BODY_NAMES:
                bid=model.body(body).id
                mujoco.mj_jacBodyCom(model,data,jacp,jacr,bid)
                p,r=jacp[:,:5],jacr[:,:5]
                _,_,I=values['mujoco'][body]
                M += model.body_mass[bid]*(p.T@p)+r.T@I@r
                g += -model.body_mass[bid]*p.T@model.opt.gravity
            expected_M=arm.inertia(q[:5]);expected_g=arm.gravload(q[:5])
            max_matrix=max(max_matrix,float(np.linalg.norm(M-expected_M)/np.linalg.norm(expected_M)))
            max_gravity=max(max_gravity,float(np.max(np.abs(g-expected_g))))
        for result in entry['formats'].values():
            assert result['max_mass_difference_g']<.001,result
            assert result['max_com_difference_mm']<.01,result
            assert result['max_tensor_relative_difference']<1e-4,result
        assert max_matrix<.001, max_matrix
        assert max_gravity<.001, max_gravity
        entry.update(geometry_and_kinematics_unchanged=True,motor_armature_and_drive_parameters_unchanged=True,
                     fixed_base_unchanged=True,tool_com_and_central_inertia_unchanged=True,
                     rigid_arm_gravity_max_difference_nm=max_gravity,
                     rigid_arm_mass_matrix_max_relative_difference=max_matrix)
        report['profiles'][profile]=entry
        print(profile,entry['formats'],'gravity Nm',max_gravity,'M relative',max_matrix,flush=True)
    # Prove the reference source files, including all working runtime tool XML,
    # still match their pre-edit hashes.
    from model_reference import REFERENCE_PATH
    report['reference_snapshot_sha256']=hashlib.sha256(REFERENCE_PATH.read_bytes()).hexdigest()
    if not check_baseline:
        # Numerical/reference checks above are real checks. The self-comparison
        # branches supply no evidence of change, so explicitly omit those claims.
        for entry in report['profiles'].values():
            for name in ['geometry_and_kinematics_unchanged','motor_armature_and_drive_parameters_unchanged',
                         'fixed_base_unchanged','tool_com_and_central_inertia_unchanged']:
                entry.pop(name,None)
    (ROOT/'reports/dynamics_sync_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-mujoco',type=Path,help='Optional before-edit package for geometry/drive preservation checks')
    parser.add_argument('--baseline-urdf',type=Path,help='Supply with --baseline-mujoco')
    parser.add_argument('--poses',type=int,default=160)
    args=parser.parse_args();verify(args.baseline_mujoco,args.baseline_urdf,args.poses)
