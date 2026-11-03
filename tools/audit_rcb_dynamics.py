"""Compare the captured RCB reference with URDF/MJCF in the same DH frames.

Reads the versioned reference snapshot; no RCB/application checkout is needed.
Use import_rcb_reference.py explicitly to refresh from a live runtime checkout.
"""

from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from model_reference import (ROOT,REFERENCE_PATH,REFERENCE,RCB_FROM_MJ_OFFSET,
                             ARM_JOINTS,GRIPPER_FOR_VARIANT,
                             load_reference_robot as load_rcb_robot)

MJCF=ROOT/'mujoco/par6'
URDF=ROOT/'urdf/par6_description'
BODY_NAMES=['shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link']


def origin(element):
    T=np.eye(4)
    if element is not None:
        T[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return T


def urdf_bodies(path,q_rcb,finger_position=0,return_poses=False):
    tree=ET.parse(path).getroot()
    links={e.get('name'):e for e in tree.findall('link')}
    joints=tree.findall('joint')
    children={j.find('child').get('link') for j in joints}
    poses={next(name for name in links if name not in children):np.eye(4)}
    values=dict(zip(ARM_JOINTS,q_rcb-RCB_FROM_MJ_OFFSET))
    values.update(left_finger_joint=-finger_position,right_finger_joint=finger_position)
    pending=list(joints)
    while pending:
        progress=False
        for joint in list(pending):
            parent=joint.find('parent').get('link')
            if parent not in poses:continue
            motion=np.eye(4)
            kind=joint.get('type')
            if kind!='fixed':
                axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                value=values.get(joint.get('name'),0.)
                if kind=='prismatic':motion[:3,3]=axis*value
                else:motion[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix()
            poses[joint.find('child').get('link')]=poses[parent]@origin(joint.find('origin'))@motion
            pending.remove(joint);progress=True
        if not progress:raise ValueError('Disconnected URDF joint tree')
    if return_poses:return poses
    bodies={}
    for name,link in links.items():
        inertial=link.find('inertial')
        if inertial is None:continue
        mass=float(inertial.find('mass').get('value'))
        if mass==0:continue
        I=inertial.find('inertia')
        tensor=np.array([[float(I.get('ixx')),float(I.get('ixy')),float(I.get('ixz'))],
                         [float(I.get('ixy')),float(I.get('iyy')),float(I.get('iyz'))],
                         [float(I.get('ixz')),float(I.get('iyz')),float(I.get('izz'))]])
        T=poses[name]@origin(inertial.find('origin'))
        bodies[name]=(mass,T[:3,3],T[:3,:3]@tensor@T[:3,:3].T)
    return bodies


def mjcf_bodies(model,data,q_rcb,finger_position=0):
    data.qpos[:6]=q_rcb-RCB_FROM_MJ_OFFSET
    if model.nq>6:
        data.qpos[model.joint('left_finger_joint').qposadr]=-finger_position
        data.qpos[model.joint('right_finger_joint').qposadr]=finger_position
    mujoco.mj_forward(model,data)
    return {model.body(i).name:(float(model.body_mass[i]),data.xipos[i].copy(),
            data.ximat[i].reshape(3,3)@np.diag(model.body_inertia[i])@data.ximat[i].reshape(3,3).T)
            for i in range(1,model.nbody) if model.body_mass[i]>0}


def aggregate(bodies,names,T):
    selected=[bodies[name] for name in names if name in bodies]
    mass=sum(item[0] for item in selected)
    if mass==0:return {'mass_kg':0.,'com_m':[0.,0.,0.],'inertia_kg_m2':np.zeros((3,3)).tolist()}
    com=sum(m*p for m,p,_ in selected)/mass
    inertia=np.zeros((3,3))
    for m,p,I in selected:
        r=p-com
        inertia+=I+m*((r@r)*np.eye(3)-np.outer(r,r))
    R=T[:3,:3]
    return {'mass_kg':mass,'com_m':(R.T@(com-T[:3,3])).tolist(),
            'inertia_kg_m2':(R.T@inertia@R).tolist(),'bodies':names}


def difference(reference,actual):
    A=np.asarray(reference['inertia_kg_m2']);B=np.asarray(actual['inertia_kg_m2'])
    return {'mass_difference_g':(actual['mass_kg']-reference['mass_kg'])*1000,
            'com_distance_mm':float(np.linalg.norm(np.asarray(actual['com_m'])-reference['com_m'])*1000),
            'inertia_frobenius_relative_error':float(np.linalg.norm(B-A)/np.linalg.norm(A)) if np.linalg.norm(A)>0 else None}


def audit():
    sources=[REFERENCE_PATH]
    measurements_path=ROOT/'source/dynamics/physical_measurements.json'
    measurements=json.loads(measurements_path.read_text(encoding='utf-8')) if measurements_path.exists() else None
    report={'reference':'RCB as currently configured, not modified. User prefers its arm link mass/COM/central inertia; motor rotor inertias are uncalibrated estimates.',
            'sources':{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
            'reference_capture_date':REFERENCE['captured_date'],
            'upstream_source_hashes':REFERENCE['upstream_files_sha256'],
            'frame_convention':'All COM and central inertia tensors expressed in the corresponding RCB DH link frame. Tool includes moving fingers and fixed tool bodies; static base excluded.',
            'physical_measurements':measurements,
            'caveats':['RCB combines the end effector into one rigid link; its reference jaw opening is not recorded.',
                       'Rotor/gearbox inertia is compared separately from rigid-body inertia.',
                       'User reports current RCB motor rotor inertias are incorrect; calibration is deferred. They are not measured reference values.',
                       'User-reported tool masses in physical_measurements supersede the older configured tool masses; no model values are changed by this audit.',
                       'No reference or simulation inertial values were changed by this audit.'],
            'variants':{}}
    q=RCB_FROM_MJ_OFFSET.copy()
    for variant,tool in GRIPPER_FOR_VARIANT.items():
        robot=load_rcb_robot(tool)
        frames=np.asarray(robot.fkine_all(q).A)
        model=mujoco.MjModel.from_xml_path(str(MJCF/f'scene_{variant}.xml'))
        data=mujoco.MjData(model)
        bodies={'mujoco':mjcf_bodies(model,data,q),'urdf':urdf_bodies(URDF/f'par6_{variant}.urdf',q)}
        details=[]
        for i,link in enumerate(robot.links):
            reference={'mass_kg':float(link.m),'com_m':np.asarray(link.r).tolist(),
                       'inertia_kg_m2':np.asarray(link.I).tolist()}
            entry={'link':f'J{i+1}' if i<5 else 'complete_tool','rcb':reference}
            for format_name,items in bodies.items():
                names=[BODY_NAMES[i]] if i<5 else [name for name in items if name not in BODY_NAMES+['base_link']]
                actual=aggregate(items,names,frames[i+1])
                entry[format_name]=actual
                entry[format_name+'_difference']=difference(reference,actual)
            entry['urdf_vs_mujoco']=difference(entry['mujoco'],entry['urdf'])
            details.append(entry)
        max_stroke=.0525 if variant=='msg' else .0235 if variant=='ssg48' else 0
        closed={}
        if max_stroke:
            for format_name,items in [('mujoco',mjcf_bodies(model,data,q,max_stroke)),
                                       ('urdf',urdf_bodies(URDF/f'par6_{variant}.urdf',q,max_stroke))]:
                names=[name for name in items if name not in BODY_NAMES+['base_link']]
                actual=aggregate(items,names,frames[-1])
                closed[format_name]=actual
                closed[format_name+'_difference']=difference(details[-1]['rcb'],actual)
        # Compare effective arm dynamics over poses as a second check on frame conversion.
        rng=np.random.default_rng(2710)
        gravity_errors=[];mass_errors=[];rigid_mass_errors=[]
        rcb_rotor=np.array([float(link.Jm*link.G**2) for link in robot.links])
        tcp_errors={'mujoco':[],'urdf':[]}
        for sample in [q,*[q+rng.uniform(-.6,.6,6) for _ in range(24)]]:
            mjcf_bodies(model,data,sample)
            reference_tcp=np.asarray(robot.fkine(sample).A)
            native_tcp=np.eye(4);native_tcp[:3,3]=data.site('tcp').xpos
            native_tcp[:3,:3]=data.site('tcp').xmat.reshape(3,3)
            urdf_tcp=urdf_bodies(URDF/f'par6_{variant}.urdf',sample,return_poses=True)['tcp']
            for kind,pose in [('mujoco',native_tcp),('urdf',urdf_tcp)]:
                tcp_errors[kind].append([float(np.linalg.norm(pose[:3,3]-reference_tcp[:3,3])),
                    float(Rotation.from_matrix(pose[:3,:3]@reference_tcp[:3,:3].T).magnitude())])
            gravity_errors.append((data.qfrc_bias[:6]-np.asarray(robot.gravload(sample))).tolist())
            matrix=np.zeros((model.nv,model.nv));mujoco.mj_fullM(model,matrix,data.qM)
            rcb_matrix=np.asarray(robot.inertia(sample))
            mass_errors.append(float(np.linalg.norm(matrix[:6,:6]-rcb_matrix)/np.linalg.norm(rcb_matrix)))
            rigid_rcb=rcb_matrix-np.diag(rcb_rotor)
            rigid_mj=matrix[:6,:6]-np.diag(model.dof_armature[:6])
            rigid_mass_errors.append(float(np.linalg.norm(rigid_mj-rigid_rcb)/np.linalg.norm(rigid_rcb)))
        report['variants'][variant]={'rcb_tool':tool,'links':details,'tool_fully_closed':closed,
            'rotor_armature_rcb_kg_m2':[float(link.Jm*link.G**2) for link in robot.links],
            'rotor_armature_mujoco_kg_m2':model.dof_armature[:6].tolist(),
            'tcp_max_error':{kind:dict(zip(['position_m','angle_rad'],np.max(values,axis=0).tolist())) for kind,values in tcp_errors.items()},
            'limits':{'rcb_qlim_deg':np.degrees(robot.qlim).tolist(),
                      'mujoco_arm_range_rcb_deg':np.degrees(model.jnt_range[:6]+RCB_FROM_MJ_OFFSET[:,None]).tolist(),
                      'urdf_arm_joint_types':[ET.parse(URDF/f'par6_{variant}.urdf').find(f'joint[@name="{name}"]').get('type') for name in ARM_JOINTS]},
            'poses_checked':25,'max_gravity_torque_difference_nm':np.max(np.abs(gravity_errors),axis=0).tolist(),
            'mass_matrix_max_relative_difference':max(mass_errors),
            'mass_matrix_without_rotor_max_relative_difference':max(rigid_mass_errors),
            'rotor_reference_status':'Uncalibrated estimates; correction deferred by user'}
    output=ROOT/'reports/dynamics';output.mkdir(parents=True,exist_ok=True)
    (output/'rcb_model_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    lines=['# RCB / MuJoCo / URDF mass-property comparison','',
           'RCB arm link mass, COM and central inertia are the user-preferred reference. This report changes no dynamics data.',
           'Motor rotor inertias in RCB and simulation are uncalibrated estimates; the user has deferred correcting them.',
           'COM and inertia are transformed into matching DH frames before comparison.','',
           f'Reference: the versioned `source/dynamics/rcb_reference.json` snapshot captured {REFERENCE["captured_date"]}. Builds do not import a local runtime or motion application.',
           'Upstream arm data originates in `robots/PAR6.py`, tool data in `grippers/*.xml`, and limits in `robots/PAR6.xml`. Source hashes are recorded in the snapshot. Refresh only with `tools/import_rcb_reference.py --rcb-root PATH --date YYYY-MM-DD`.',
           'The live dynamics builder is `utility/pinocchio_attach.py::build_pinocchio_dynamics`; it consumes those RTB link mass/COM/inertia values and reflected rotor inertia. A separate kinematic helper uses placeholder inertias and is not the measured dynamics reference.','']
    if measurements:
        lines += ['## Physical mass updates reported 2026-10-28','',
                  'Physical measurements are recorded in `source/dynamics/physical_measurements.json`. The tables compare the currently configured RCB and exported model values.',
                  f'Application status: {measurements["status"]} Your RCB runtime files are read-only for this export.','',
                  '| Tool | Physical total, g | Each moving assembly, g | Fixed assembly, g |',
                  '|---|---:|---:|---:|']
        for tool in ['flange','msg','ssg48']:
            item=measurements['tools'][tool]
            moving=item.get('left_moving_assembly_mass_kg',item.get('moving_assembly_mass_kg'))
            fixed=item.get('fixed_assembly_mass_kg')
            lines.append(f'| {tool} | {item["total_mass_kg"]*1000:.1f} | {"pending" if moving is None else f"{moving*1000:.1f}"} | {"n/a or pending" if fixed is None else f"{fixed*1000:.1f}"} |')
        lines += ['','MSG moving assemblies include finger, jaw and sliding elements; the 39.3 g figure is approximate.','']
    for variant,item in report['variants'].items():
        lines += [f'## {variant} ({item["rcb_tool"]})','',
                  '| Link | RCB mass g | MJCF mass g | URDF mass g | MJCF COM delta mm | URDF COM delta mm | MJCF inertia relative delta | URDF inertia relative delta |',
                  '|---|---:|---:|---:|---:|---:|---:|---:|']
        for entry in item['links']:
            errors=[entry[f'{key}_difference'] for key in ['mujoco','urdf']]
            inertia=['n/a' if v['inertia_frobenius_relative_error'] is None else f'{v["inertia_frobenius_relative_error"]:.1%}' for v in errors]
            lines.append(f'| {entry["link"]} | {entry["rcb"]["mass_kg"]*1000:.3f} | {entry["mujoco"]["mass_kg"]*1000:.3f} | {entry["urdf"]["mass_kg"]*1000:.3f} | {errors[0]["com_distance_mm"]:.3f} | {errors[1]["com_distance_mm"]:.3f} | {inertia[0]} | {inertia[1]} |')
        lines += ['',f'Largest gravity torque differences over 25 poses, J1-J6 (Nm): `{np.round(item["max_gravity_torque_difference_nm"],3).tolist()}`.',
                  f'Largest relative arm mass-matrix difference: {item["mass_matrix_max_relative_difference"]:.1%}.',
                  'This includes the existing estimated rotor/armature settings; see JSON for both arrays.',
                  f'With rotor terms removed from both models, the largest relative arm mass-matrix difference is {item["mass_matrix_without_rotor_max_relative_difference"]:.1%}.','']
        lines += [f'TCP maximum position/angle error against RCB: `{item["tcp_max_error"]}` (metres/radians).',
                  'Joint-limit arrays are in the JSON. Portable URDF arm joints remain continuous and native MJCF ranges are permissive simulation ranges; neither is a copy of the RCB hardware limits.','']
    lines+=['## Interpretation','',
            '- Version 2.8 synchronizes arm link properties and measured simulation tool masses. RCB runtime tool masses remain at their older configured values.',
            '- Whole-robot discrepancies below include unresolved tool COM/inertia and estimated rotor terms. The separate dynamics_sync_validation.json isolates and validates the five-link arm without these terms.',
            '- Close mass values alone do not establish matching COM or inertia.',
            '- RCB tool inertia is aggregated; simulation fingers move. The reference tool opening is unspecified.',
            '- A dynamics synchronization should convert RCB DH inertias into each CAD body frame, account for fixed/moving tool aggregation, and validate gravity and mass matrices. Raw tensor copying is incorrect.',
            '- Measured-reference data in RCB-Runtime was not edited.','']
    (output/'rcb_model_comparison.md').write_text('\n'.join(lines),encoding='utf-8')
    print('\n'.join(lines[:12]))
    for name,item in report['variants'].items():
        print(name,'gravity Nm',np.round(item['max_gravity_torque_difference_nm'],3).tolist(),
              'mass-matrix relative delta',round(item['mass_matrix_max_relative_difference'],4))


if __name__=='__main__':audit()
