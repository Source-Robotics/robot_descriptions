"""Sweep both SSG48 jaw options for unintended internal collision contacts.

The URDF check imports its actual collision meshes into MuJoCo, with adjacent
body filtering. It does not establish the collision policy of every URDF
consumer. Internal transmissions are represented by the coupled finger joints,
not by rack/gear contact simulation.
"""

from pathlib import Path
import json
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parent))

import mujoco
import numpy as np

from split_motor_visuals import ROOT


def collision_model(robot,profile,format_name):
    if format_name=='mujoco':
        return mujoco.MjModel.from_xml_path(str(ROOT/f'mujoco/{robot}/scene_{profile}.xml'))
    path=ROOT/f'urdf/{robot}_description/{robot}_{profile}.urdf'
    root=ET.parse(path).getroot()
    for mesh in root.findall('.//mesh'):
        mesh.set('filename',(path.parent/mesh.get('filename')).as_posix())
    # Keep named finger bodies so this probes exactly the collision shapes
    # distributed to URDF consumers, rather than the native package's meshes.
    settings=root.find('mujoco')
    if settings is None:settings=ET.SubElement(root,'mujoco')
    compiler=settings.find('compiler')
    if compiler is None:compiler=ET.SubElement(settings,'compiler')
    compiler.attrib.update(discardvisual='true',fusestatic='false',strippath='false')
    for link in root.findall('link'):
        for visual in link.findall('visual'):link.remove(visual)
        for index,geom in enumerate(link.findall('collision')):
            geom.set('name',f'{link.get("name")}_collision_{index}')
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))


def sweep(model,samples=201):
    data=mujoco.MjData(model)
    internal={model.body(name).id for name in
              ['ssg48_gripper_link','left_finger_link','right_finger_link']}
    travel=-float(model.joint('left_finger_joint').range[0])
    deepest=0.;at_travel=0.;pair=None
    for value in np.linspace(0.,travel,samples):
        data.joint('left_finger_joint').qpos[:]=-value
        data.joint('right_finger_joint').qpos[:]=value
        mujoco.mj_forward(model,data)
        for contact in data.contact:
            bodies=[int(model.geom_bodyid[g]) for g in contact.geom]
            if all(body in internal for body in bodies) and -contact.dist>deepest:
                deepest=-float(contact.dist);at_travel=float(value)
                pair=[model.body(body).name for body in bodies]
    return {'travel_samples':samples,'travel_per_finger_m':travel,
            'maximum_internal_penetration_m':deepest,
            'at_travel_m':at_travel,'deepest_body_pair':pair,
            'passed':deepest<1e-6}


def verify():
    results=[]
    for robot in ['par6','parol6']:
        for profile in ['ssg48','ssg48_stock']:
            for kind in ['mujoco','urdf']:
                result={'robot':robot,'profile':profile,'format':kind,
                        **sweep(collision_model(robot,profile,kind))}
                assert result['passed'],result
                results.append(result)
    report={'scope':'MuJoCo native and imported URDF collision meshes, with adjacent-body filtering. Housing/finger overlap in a slide joint is not a simulated transmission contact.',
            'ssg48_collision_policy':'Keep the complete rack visual; use the rack-free external carriage/jaw contact envelope in both formats.',
            'results':results}
    (ROOT/'reports/ssg48_internal_contacts.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('SSG48: eight robot/jaw/format combinations passed 201-point internal contact sweeps.',flush=True)
    return report


if __name__=='__main__':verify()
