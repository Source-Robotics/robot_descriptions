"""Explicitly refresh the portable reference snapshot from an RCB checkout.

Normal builds never import RCB. Review the resulting data diff before release.
"""

from pathlib import Path
import argparse
import hashlib
import json
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
VARIANTS={'flange':'Flange','msg':'MSG_small_motor_150mm_rail','ssg48':'SSG48'}


def capture(rcb_root,date):
    rcb_root=Path(rcb_root).resolve()
    if not (rcb_root/'robots/PAR6.py').is_file():raise ValueError('--rcb-root must contain robots/PAR6.py')
    sys.path.insert(0,str(rcb_root))
    from robots.PAR6 import PAR6
    paths=['robots/PAR6.py','robots/PAR6.xml','utility/pinocchio_attach.py']+[f'grippers/{tool}.xml' for tool in VARIANTS.values()]
    data={'schema_version':1,'captured_date':date,'upstream':'RCB-Runtime',
          'upstream_files_sha256':{name:hashlib.sha256((rcb_root/name).read_bytes()).hexdigest() for name in paths},
          'status':'Arm link mass/COM/central inertia preferred by user. Tool data is the captured runtime configuration; rotor values are uncalibrated estimates.',
          'rcb_from_mj_offset_rad':[0.,-np.pi/2,np.pi,0.,0.,np.pi],
          'arm_joint_names':['base_joint','shoulder_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
          'variants':{}}
    checks={'captured_date':date,'source':'Live RCB/Robotics Toolbox outputs; regression fixtures for the portable evaluator','cases':[]}
    rng=np.random.default_rng(3110)
    for variant,tool in VARIANTS.items():
        robot=PAR6(symbolic=False,gripper_name=tool)
        if any(link.mdh or link.sigma for link in robot.links):raise ValueError('Only standard revolute DH chains are supported')
        links=[]
        for link in robot.links:
            links.append({'d':float(link.d),'a':float(link.a),'alpha':float(link.alpha),'offset':float(link.offset),
                          'mass_kg':float(link.m),'com_m':np.asarray(link.r).tolist(),
                          'central_inertia_kg_m2':np.asarray(link.I).tolist(),
                          'motor_rotor_inertia_kg_m2':float(link.Jm),'gear_ratio':float(link.G)})
        data['variants'][variant]={'tool_name':tool,'links':links,'base_transform':np.asarray(robot.base.A).tolist(),
            'tool_transform':np.asarray(robot.tool.A).tolist(),'gravity_m_s2':np.asarray(robot.gravity).tolist(),
            'software_joint_limits_rad':np.asarray(robot.qlim).tolist()}
        for q in [np.asarray(data['rcb_from_mj_offset_rad']),*[rng.uniform(robot.qlim[0],robot.qlim[1]) for _ in range(3)]]:
            checks['cases'].append({'tool_name':tool,'q_rad':q.tolist(),'tcp_pose':np.asarray(robot.fkine(q).A).tolist(),
                'mass_matrix':np.asarray(robot.inertia(q)).tolist(),'gravity_torque_nm':np.asarray(robot.gravload(q)).tolist()})
    output=ROOT/'source/dynamics/rcb_reference.json'
    output.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8',newline='\n')
    (output.parent/'rcb_reference_checks.json').write_text(json.dumps(checks,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(f'Captured portable reference: {output.name}; review the changed data before rebuilding.')
    return data


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rcb-root',type=Path,required=True)
    parser.add_argument('--date',required=True,help='Capture date, YYYY-MM-DD')
    args=parser.parse_args();capture(args.rcb_root,args.date)
