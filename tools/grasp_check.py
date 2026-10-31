"""Check close, lift, hold and release on the four packaged jaw profiles."""

from pathlib import Path
import argparse
import json
import sys
import time

import mujoco
import numpy as np

from split_motor_visuals import MJCF

PROFILES=['msg-solid','msg-soft-tpu','ssg48-solid','ssg48-fin-ray']


def validate_initial_placement(model,data,tolerance=1e-6):
    """Reject a fixture that starts with its object inside another solid."""
    mujoco.mj_forward(model,data)
    object_id=model.body('test_object').id
    for contact in data.contact:
        bodies=[int(model.geom_bodyid[g]) for g in contact.geom]
        if object_id in bodies and contact.dist < -tolerance:
            names=[model.geom(g).name or model.body(model.geom_bodyid[g]).name for g in contact.geom]
            raise ValueError(f'Invalid initial object placement: {names}, penetration {-contact.dist*1000:.3f} mm')


def run_case(profile,shape='box',view=False,preview=None,robot='par6'):
    if robot not in ['par6','parol6']:raise ValueError(f'Unknown robot: {robot}')
    folder=MJCF.parent/robot/'bench'
    meta=json.loads((folder/'profiles.json').read_text())[profile]
    model=mujoco.MjModel.from_xml_path(str(folder/f'{profile}_{shape}.xml'))
    data=mujoco.MjData(model)
    validate_initial_placement(model,data)
    dt=model.opt.timestep
    initial_mocap=data.mocap_pos.copy()
    object_id=model.body('test_object').id
    geom_id=model.geom('object_surface').id
    gripper=model.actuator('gripper_actuator').id
    close=float(model.actuator_ctrlrange[gripper,0])
    initial_z=None;hold_z=None;release_z=None;contacts=set()
    renderer=None;viewer=None;frames=[];internal_penetration=0.
    internal_ids={model.body(meta['tool_body']).id,
                  model.body('left_finger_link').id,model.body('right_finger_link').id}
    if preview:
        model.vis.global_.offwidth,model.vis.global_.offheight=900,700
        renderer=mujoco.Renderer(model,width=900,height=700)
        camera=mujoco.MjvCamera();camera.lookat[:]=[meta['object_center_x_m'],0,meta['table_top_m']+.09]
        camera.distance,camera.azimuth,camera.elevation=.40,135,-20
    if view:
        from mujoco import viewer as mj_viewer
        viewer=mj_viewer.launch_passive(model,data)
        viewer.cam.lookat[:]=[0,0,meta['table_top_m']+.09]
        viewer.cam.distance,viewer.cam.azimuth,viewer.cam.elevation=.45,135,-20
    checkpoints=[1.8,3.6,4.9]
    try:
        for step in range(int(5.0/dt)):
            t=step*dt
            data.ctrl[gripper]=close if .4<=t<3.8 else 0.
            lift=np.clip((t-1.9)/1.0,0,1)*.05
            data.mocap_pos[:]=initial_mocap
            data.mocap_pos[:,2]+=lift
            mujoco.mj_step(model,data)
            if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))):raise RuntimeError('Non-finite grasp state')
            for contact in data.contact:
                if all(model.geom_bodyid[g] in internal_ids for g in contact.geom):
                    internal_penetration=max(internal_penetration,-float(contact.dist))
            if initial_z is None and t>=.35:initial_z=float(data.xpos[object_id,2])
            if 3.4<=t<3.8:
                hold_z=float(data.xpos[object_id,2])
                for contact in data.contact:
                    if geom_id not in contact.geom:continue
                    other=int(contact.geom[1] if contact.geom[0]==geom_id else contact.geom[0])
                    name=model.body(model.geom_bodyid[other]).name
                    if 'finger' in name:contacts.add(name)
            if t>=4.9:release_z=float(data.xpos[object_id,2])
            if renderer and checkpoints and t>=checkpoints[0]:
                renderer.update_scene(data,camera)
                frames.append(renderer.render().copy());checkpoints.pop(0)
            if viewer:
                if not viewer.is_running():break
                viewer.sync();time.sleep(dt)
    finally:
        if renderer:renderer.close()
        if viewer:viewer.close()
    if preview and frames:
        from PIL import Image,ImageDraw
        result=Image.new('RGB',(900*len(frames),740),'#17202c')
        drawing=ImageDraw.Draw(result)
        for i,(frame,label) in enumerate(zip(frames,['Closed','Lift and hold','Released'])):
            result.paste(Image.fromarray(frame),(i*900,40));drawing.text((i*900+20,12),label,fill='white')
        Path(preview).parent.mkdir(parents=True,exist_ok=True);result.save(preview)
    lifted=None if hold_z is None or initial_z is None else hold_z-initial_z
    dropped=None if hold_z is None or release_z is None else hold_z-release_z
    warnings=int(np.sum(data.warning.number))
    passed=(lifted is not None and lifted>.035 and dropped is not None and dropped>.025 and len(contacts)==2
            and internal_penetration<1e-6 and warnings==0)
    return {'robot':robot,'profile':profile,'shape':shape,'object_mass_kg':meta['object_mass_kg'],
            'object_height_m':meta['object_height_m'],
            'lifted_m':lifted,'release_drop_m':dropped,'contacting_fingers':sorted(contacts),'passed':passed,
            'maximum_internal_penetration_m':internal_penetration,'simulation_warnings':warnings,
            'scope':'Simulation regression fixture; force/friction/compliance are not hardware-calibrated.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',choices=PROFILES,default='ssg48-solid')
    parser.add_argument('--shape',choices=['box','cylinder'],default='box')
    parser.add_argument('--robot',choices=['par6','parol6'],default='par6')
    parser.add_argument('--all',action='store_true')
    parser.add_argument('--view',action='store_true')
    parser.add_argument('--preview',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    cases=[(p,s) for p in PROFILES for s in ['box','cylinder']] if args.all else [(args.profile,args.shape)]
    results=[run_case(p,s,args.view,args.preview if len(cases)==1 else None,args.robot) for p,s in cases]
    text=json.dumps(results,indent=2)
    print(text)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text+'\n')
    if not all(item['passed'] for item in results):raise SystemExit(1)


if __name__=='__main__':main()
