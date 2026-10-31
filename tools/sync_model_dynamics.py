"""Export measured arm properties and tool masses without changing geometry.

RCB is read-only. Motor armature and fixed-base properties are untouched.
Tool COM and central inertia remain explicit, unverified existing estimates.
"""

from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from audit_rcb_dynamics import (ROOT, MJCF, URDF, RCB_FROM_MJ_OFFSET,
                                load_rcb_robot, urdf_bodies)
from model_reference import REFERENCE_PATH

SOURCE=ROOT/'source/dynamics'


def numbers(values):
    return ' '.join(format(float(value),'.17g') for value in np.ravel(values))


def body_properties(reference, world_dh, world_body):
    """Convert COM and COM-centred inertia from DH frame to CAD body frame."""
    dh_body=np.linalg.inv(world_dh)@world_body
    rotation=dh_body[:3,:3]
    com=rotation.T@(np.asarray(reference['com_m'])-dh_body[:3,3])
    inertia=rotation.T@np.asarray(reference['inertia_kg_m2'])@rotation
    return com,(inertia+inertia.T)/2,dh_body


def write_native(inertial,mass,com,inertia):
    # Columns are principal axes expressed in the CAD body frame.
    values,axes=np.linalg.eigh(inertia)
    if values.min()<=0 or values[-1]>values[:2].sum()+1e-12:
        raise ValueError('Non-physical central inertia in RCB reference')
    if np.linalg.det(axes)<0:axes[:,0]*=-1
    quaternion=Rotation.from_matrix(axes).as_quat()
    inertial.attrib.clear()
    inertial.set('mass',format(mass,'.17g'))
    inertial.set('pos',numbers(com))
    inertial.set('quat',numbers(np.r_[quaternion[3],quaternion[:3]]))
    inertial.set('diaginertia',numbers(values))


def write_urdf(inertial,mass,com,inertia):
    inertial.find('mass').set('value',format(mass,'.17g'))
    origin=inertial.find('origin')
    if origin is None:origin=ET.SubElement(inertial,'origin')
    origin.set('xyz',numbers(com));origin.set('rpy','0 0 0')
    tensor=inertial.find('inertia')
    for key,row,col in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]:
        tensor.set(key,format(float(inertia[row,col]),'.17g'))


def tool_masses(family,measurements):
    item=measurements['tools'][family]
    if family=='flange':return {'flange_link':item['total_mass_kg']}
    result={f'{family}_gripper_link':item['fixed_assembly_mass_kg'],
            'left_finger_link':item['left_moving_assembly_mass_kg'],
            'right_finger_link':item['right_moving_assembly_mass_kg']}
    if not np.isclose(sum(result.values()),item['total_mass_kg'],rtol=0,atol=1e-12):
        raise ValueError(f'{family}: measured component masses do not sum to total')
    return result


def synchronize():
    mapping=json.loads((SOURCE/'model_mapping.json').read_text())
    measurements=json.loads((SOURCE/'physical_measurements.json').read_text())
    if not mapping['enabled']:raise ValueError('Dynamics export is disabled')
    robot=load_rcb_robot('Flange')
    q=RCB_FROM_MJ_OFFSET.copy()
    frames=np.asarray(robot.fkine_all(q).A)
    references=[{'body':body,'mass_kg':float(link.m),'com_m':np.asarray(link.r).tolist(),
                 'inertia_kg_m2':np.asarray(link.I).tolist()}
                for body,link in zip(mapping['arm_bodies'],robot.links[:5])]
    sources=[REFERENCE_PATH,SOURCE/'physical_measurements.json',SOURCE/'model_mapping.json']
    report={'scope':'RCB arm link properties + user-measured tool masses. RCB runtime is read-only.',
            'sources_sha256':{path.relative_to(ROOT).as_posix():hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
            'reference_q_rcb_rad':q.tolist(),'arm_reference_dh':references,
            'tool_com_and_central_inertia':'Existing values preserved as unverified estimates; not recomputed from mass.',
            'motor_rotor_inertia':'Existing consumer estimates unchanged; deferred.',
            'fixed_base':'Unchanged; excluded from moving-arm synchronization.',
            'profiles':{}}
    for profile,family in mapping['profiles'].items():
        masses=tool_masses(family,measurements)
        entry={'measured_tool_masses_kg':masses,'formats':{}}
        for kind,package,extension in [('mujoco',MJCF,'xml'),('urdf',URDF,'urdf')]:
            path=package/f'par6_{profile}.{extension}'
            tree=ET.parse(path);root=tree.getroot()
            if kind=='mujoco':
                model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model)
                data.qpos[:6]=0;mujoco.mj_forward(model,data)
                poses={}
                for body in mapping['arm_bodies']:
                    bid=model.body(body).id
                    pose=np.eye(4);pose[:3,3]=data.xpos[bid];pose[:3,:3]=data.xmat[bid].reshape(3,3)
                    poses[body]=pose
                inertial=lambda name:root.find(f'.//body[@name="{name}"]/inertial')
            else:
                poses=urdf_bodies(path,q,return_poses=True)
                inertial=lambda name:root.find(f'link[@name="{name}"]/inertial')
            transforms={}
            for index,reference in enumerate(references):
                body=reference['body']
                com,inertia,transform=body_properties(reference,frames[index+1],poses[body])
                element=inertial(body)
                if element is None:raise ValueError(f'{profile}/{kind}: missing inertial for {body}')
                if kind=='mujoco':write_native(element,reference['mass_kg'],com,inertia)
                else:write_urdf(element,reference['mass_kg'],com,inertia)
                transforms[body]={'dh_from_body':transform.tolist(),'com_body_m':com.tolist(),
                                  'central_inertia_body_kg_m2':inertia.tolist()}
            for body,mass in masses.items():
                element=inertial(body)
                if element is None:raise ValueError(f'{profile}/{kind}: missing tool inertial for {body}')
                if kind=='mujoco':element.set('mass',format(mass,'.17g'))
                else:element.find('mass').set('value',format(mass,'.17g'))
            ET.indent(tree,space='  ');tree.write(path,encoding='utf-8',xml_declaration=True)
            entry['formats'][kind]=transforms
        report['profiles'][profile]=entry
        print(f'{profile}: synchronized five arm bodies; tool total {sum(masses.values())*1000:.1f} g',flush=True)
    (SOURCE/'rcb_arm_reference.generated.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    synchronize()
    from build_grasp_benches import build
    build()
