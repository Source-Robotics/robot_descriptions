"""Portable NumPy implementation of the captured standard-DH reference.

No application, RCB runtime or robotics-toolbox imports. Data is versioned in
source/dynamics/rcb_reference.json; refresh it explicitly with the import tool.
"""

from pathlib import Path
from types import SimpleNamespace
import json

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
REFERENCE_PATH=ROOT/'source/dynamics/rcb_reference.json'
REFERENCE=json.loads(REFERENCE_PATH.read_text(encoding='utf-8'))
RCB_FROM_MJ_OFFSET=np.asarray(REFERENCE['rcb_from_mj_offset_rad'])
ARM_JOINTS=tuple(REFERENCE['arm_joint_names'])
GRIPPER_FOR_VARIANT={key:value['tool_name'] for key,value in REFERENCE['variants'].items()}


class ReferenceRobot:
    def __init__(self,definition,arm_only=False):
        self.base=SimpleNamespace(A=np.asarray(definition['base_transform']))
        self.tool=SimpleNamespace(A=np.eye(4) if arm_only else np.asarray(definition['tool_transform']))
        self.gravity=np.asarray(definition['gravity_m_s2'])
        self.links=[]
        for item in definition['links'][:5 if arm_only else 6]:
            self.links.append(SimpleNamespace(d=item['d'],a=item['a'],alpha=item['alpha'],offset=item['offset'],
                m=item['mass_kg'],r=np.asarray(item['com_m']),I=np.asarray(item['central_inertia_kg_m2']),
                Jm=0. if arm_only else item['motor_rotor_inertia_kg_m2'],G=item['gear_ratio']))
        self.qlim=np.asarray(definition['software_joint_limits_rad'])[:,:len(self.links)]
        self._definition=definition

    def arm_only(self):
        return ReferenceRobot(self._definition,arm_only=True)

    def fkine_all(self,q):
        if len(q)!=len(self.links):raise ValueError('Wrong reference joint count')
        frames=[self.base.A.copy()]
        for value,link in zip(q,self.links):
            c,s=np.cos(value+link.offset),np.sin(value+link.offset)
            ca,sa=np.cos(link.alpha),np.sin(link.alpha)
            transform=np.array([[c,-s*ca,s*sa,link.a*c],
                                [s,c*ca,-c*sa,link.a*s],
                                [0.,sa,ca,link.d],[0.,0.,0.,1.]])
            frames.append(frames[-1]@transform)
        return SimpleNamespace(A=np.asarray(frames))

    def fkine(self,q):
        return SimpleNamespace(A=self.fkine_all(q).A[-1]@self.tool.A)

    def _terms(self,q):
        frames=self.fkine_all(q).A
        n=len(self.links);matrix=np.zeros((n,n));gravity=np.zeros(n)
        for i,link in enumerate(self.links):
            frame=frames[i+1];rotation=frame[:3,:3]
            center=frame[:3,3]+rotation@link.r
            linear=np.zeros((3,n));angular=np.zeros((3,n))
            for j in range(i+1):
                axis=frames[j,:3,2]
                linear[:,j]=np.cross(axis,center-frames[j,:3,3]);angular[:,j]=axis
            matrix+=link.m*(linear.T@linear)+angular.T@(rotation@link.I@rotation.T)@angular
            gravity-=link.m*linear.T@self.gravity
        matrix+=np.diag([link.Jm*link.G**2 for link in self.links])
        return matrix,gravity

    def inertia(self,q):return self._terms(q)[0]

    def gravload(self,q):return self._terms(q)[1]


def load_reference_robot(tool_name):
    for definition in REFERENCE['variants'].values():
        if definition['tool_name']==tool_name:return ReferenceRobot(definition)
    raise ValueError(f'Unknown tool reference {tool_name!r}')
