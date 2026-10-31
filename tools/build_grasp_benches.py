"""Export standalone, movable-gripper box/cylinder validation scenes."""

import copy
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from split_motor_visuals import ROOT, MJCF
from grasp_check import validate_initial_placement

PROFILES={'msg-solid':('msg','msg_gripper_link','MSG / Solid jaws'),
          'msg-soft-tpu':('msg_tpu_glue','msg_gripper_link','MSG / Soft TPU jaws'),
          'ssg48-solid':('ssg48','ssg48_gripper_link','SSG48 / Solid jaws'),
          'ssg48-fin-ray':('ssg48_stock','ssg48_gripper_link','SSG48 / Fin-ray jaws')}


def object_height_with_clearance(root,tool_name,top,center_x):
    """Fit the benchmark box below the actual fixed collision geometry.

    The cylinder fits inside the box's XY envelope. A global housing bounding
    box would unnecessarily shrink MSG objects because of its offset camera.
    """
    probe=copy.deepcopy(root)
    body=ET.SubElement(probe.find('worldbody'),'body',name='fit_object',
                       pos=f'{center_x:.12g} 0 {top+.030:.12g}')
    ET.SubElement(body,'freejoint',name='fit_joint')
    ET.SubElement(body,'geom',name='fit_surface',type='box',size='.012 .010 .030',mass='.05')
    probe.find('compiler').set('meshdir',(MJCF/'assets').as_posix())
    model=mujoco.MjModel.from_xml_string(ET.tostring(probe,encoding='unicode'))
    data=mujoco.MjData(model);obj=model.geom('fit_surface').id
    tool=model.body(tool_name).id
    fixed={tool}
    fingers={model.body(f'{side}_finger_link').id for side in ['left','right']}
    for index in range(tool+1,model.nbody):
        if model.body_parentid[index] in fixed and index not in fingers:fixed.add(index)
    geometry=[g for g in range(model.ngeom) if model.geom_bodyid[g] in fixed
              and (model.geom_contype[g] or model.geom_conaffinity[g])]
    def clearance(height):
        model.geom_size[obj,2]=height/2
        data.joint('fit_joint').qpos[2]=top+height/2
        mujoco.mj_forward(model,data)
        return min(mujoco.mj_geomDistance(model,data,obj,g,.1,None) for g in geometry)
    low,high=.016,.060
    if clearance(high)>=.005:return high,clearance(high)
    if clearance(low)<.005:raise ValueError(f'{tool_name}: insufficient room for a grasp object')
    for _ in range(28):
        middle=(low+high)/2
        if clearance(middle)>=.005:low=middle
        else:high=middle
    return low,clearance(low)


def build():
    directory=MJCF/'bench';directory.mkdir(exist_ok=True)
    catalog={}
    for identifier,(profile,body_name,label) in PROFILES.items():
        source=ET.parse(MJCF/f'par6_{profile}.xml').getroot()
        tool=copy.deepcopy(source.find(f'.//body[@name="{body_name}"]'))
        tool.set('pos','0 0 0.25');tool.set('quat','1 0 0 0')
        tool.insert(0,ET.Element('freejoint',name='tool_free'))
        root=ET.Element('mujoco',model=f'Grasp check - {label}')
        ET.SubElement(root,'compiler',angle='radian',autolimits='true',meshdir='../assets')
        ET.SubElement(root,'option',timestep='0.002',integrator='implicitfast',gravity='0 0 -9.81')
        root.append(copy.deepcopy(source.find('default')))
        asset=ET.SubElement(root,'asset')
        used={geom.get('mesh') for geom in tool.iter('geom')}
        materials={geom.get('material') for geom in tool.iter('geom')}
        for entry in source.find('asset'):
            if (entry.tag=='mesh' and entry.get('name') in used) or (entry.tag=='material' and entry.get('name') in materials):
                asset.append(copy.deepcopy(entry))
        world=ET.SubElement(root,'worldbody')
        ET.SubElement(world,'light',pos='0 -0.3 0.8',dir='0 0 -1',directional='true')
        world.append(tool)
        ET.SubElement(world,'body',name='bench_target',mocap='true',pos='0 0 .25')
        # A mocap-mounted tool is welded to world, so MuJoCo's usual parent
        # collision filter no longer suppresses its fingers against the housing.
        contact=ET.SubElement(root,'contact')
        for side in ['left','right']:
            ET.SubElement(contact,'exclude',body1=body_name,body2=f'{side}_finger_link')
        root.append(copy.deepcopy(source.find('equality')))
        ET.SubElement(root.find('equality'),'weld',body1=body_name,body2='bench_target',solref='.005 1')
        actuator=ET.SubElement(root,'actuator')
        actuator.append(copy.deepcopy(source.find('actuator/position[@name="gripper_actuator"]')))
        # Compile the isolated tool to derive table height from its actual jaw tip.
        probe=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode').replace('../assets',(MJCF/'assets').as_posix()))
        data=mujoco.MjData(probe);mujoco.mj_forward(probe,data)
        points=[]
        for side in ['left','right']:
            bid=probe.body(f'{side}_finger_link').id
            for g in range(probe.ngeom):
                if probe.geom_bodyid[g]!=bid or probe.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH:continue
                mid=probe.geom_dataid[g];start=probe.mesh_vertadr[mid]
                vertices=probe.mesh_vert[start:start+probe.mesh_vertnum[mid]]
                points.append(vertices@data.geom_xmat[g].reshape(3,3).T+data.geom_xpos[g])
        low=float(np.vstack(points)[:,2].min())
        top=low-.008
        # Short jaws cannot accommodate the same tall object as long jaws.
        # Keep the requested 60 mm height where it fits, otherwise leave
        # 5 mm clear of the fixed housing collision surface. Do not
        # spawn the test object intersecting the gripper before closing.
        center_x=float(data.xpos[probe.body('left_finger_link').id,0])
        # A small settling gap also avoids a degenerate exact cylinder/box
        # face contact at initialization in the pinned collision engine.
        settling_gap=.0001
        height,housing_clearance=object_height_with_clearance(root,body_name,top+settling_gap,center_x)
        ET.SubElement(world,'geom',name='bench_table',type='box',pos=f'0 0 {top-.012:.9g}',
                      size='.18 .18 .012',rgba='.35 .40 .45 1',friction='1 .005 .0001')
        ET.SubElement(world,'geom',name='bench_floor',type='plane',size='.5 .5 .01',rgba='.15 .19 .25 1')
        for shape in ['box','cylinder']:
            scene=copy.deepcopy(root);w=scene.find('worldbody')
            obj=ET.SubElement(w,'body',name='test_object',pos=f'{center_x:.12g} 0 {top+settling_gap+height/2:.12g}')
            ET.SubElement(obj,'freejoint',name='object_free')
            ET.SubElement(obj,'geom',name='object_surface',type=shape,
                          size=f'.012 .010 {height/2:.9g}' if shape=='box' else f'.010 {height/2:.9g}',
                          mass='.05',rgba='.95 .48 .10 1',friction='1.2 .005 .0001',
                          solref='.005 1',solimp='.9 .95 .001')
            ET.SubElement(scene,'statistic',center=f'0 0 {top+.08:.9g}',extent='.25')
            tree=ET.ElementTree(scene);ET.indent(tree,space='  ')
            filename=f'{identifier}_{shape}.xml'
            tree.write(directory/filename,encoding='utf-8',xml_declaration=True)
            model=mujoco.MjModel.from_xml_path(str(directory/filename))
            validate_initial_placement(model,mujoco.MjData(model))
        catalog[identifier]={'label':label,'tool_body':body_name,'table_top_m':top,
                             'object_mass_kg':.05,'object_height_m':height,
                             'requested_object_height_m':.060,
                             'initial_table_gap_m':settling_gap,
                             'initial_housing_clearance_m':housing_clearance,
                             'object_center_x_m':center_x,'reference_inertias':'unchanged',
                             'note':'Simulation benchmark fixture, not measured material/force calibration.'}
    (directory/'profiles.json').write_text(json.dumps(catalog,indent=2)+'\n',encoding='utf-8')
    (directory/'README.md').write_text('''# Grasp validation scenes

Four jaw profiles, each with a 50 g box and cylinder. The dynamic gripper is
welded to a mocap target for a prescribed lift; its finger joints, contacts, material coefficients, and
inertial data come directly from the corresponding packaged robot model.
Objects are at most 60 mm tall and leave at least 5 mm below the fixed housing.
The shorter SSG48 Solid jaws therefore use approximately 41.2 mm tall objects.
Each object starts 0.1 mm above the table and settles before closing.
Both initial placement and finger self-contact are checked before accepting
a grasp result; force, friction and pass thresholds are unchanged.

From the description root, run `python tools/grasp_check.py --all` for close/lift/hold/
release checks; add `--robot parol6` to check that package's fixtures.
Use `--profile ssg48-solid --shape box --view` for an animated
MuJoCo viewer, or `--preview PATH.png` for three offscreen snapshots.

The object friction and mass are benchmark settings. Passing is a simulation
regression check and does not establish measured gripping force or TPU flex.
''',encoding='utf-8')
    print('Exported eight self-contained gripper grasp scenes.')


if __name__=='__main__':build()
