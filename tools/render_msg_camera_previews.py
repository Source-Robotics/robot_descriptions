"""Render the real named MSG camera from the existing box grasp fixtures."""
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parent))
import mujoco
from PIL import Image
from model_reference import ROOT


def render():
    destination=ROOT/'docs/images'
    destination.mkdir(parents=True,exist_ok=True)
    records=[]
    for suffix,profile in [('solid','msg-solid'),('soft_tpu','msg-soft-tpu')]:
        path=ROOT/f'mujoco/par6/bench/{profile}_box.xml'
        camera=ET.parse(path).find('.//camera[@name="msg_camera"]')
        width,height=map(int,camera.get('resolution').split())
        model=mujoco.MjModel.from_xml_path(str(path))
        model.vis.global_.offwidth=width;model.vis.global_.offheight=height
        # Presentation lighting only. Keep every camera/mesh/object/joint pose.
        model.vis.headlight.ambient[:]=[.45]*3
        model.vis.headlight.diffuse[:]=[.6]*3
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        options=mujoco.MjvOption();options.sitegroup[:]=0
        output=destination/f'msg_camera_{suffix}.png'
        with mujoco.Renderer(model,height=height,width=width) as renderer:
            renderer.update_scene(data,camera='msg_camera',scene_option=options)
            Image.fromarray(renderer.render()).save(output)
        records.append({'jaws':suffix,'source_scene':path.relative_to(ROOT).as_posix(),
            'scene_sha256':hashlib.sha256(path.read_bytes().replace(b'\r\n',b'\n')).hexdigest(),
            'camera_name':'msg_camera','camera_definition':dict(camera.attrib),
            'compiled_vertical_fov_deg':float(model.camera('msg_camera').fovy[0]),
            'image_size_px':[width,height],
            'finger_qpos_m':[float(data.joint(f'{side}_finger_joint').qpos[0]) for side in ['left','right']],
            'image':output.name,'image_sha256':hashlib.sha256(output.read_bytes()).hexdigest()})
        print(f'Rendered {output.name} from msg_camera.',flush=True)
    report={'mujoco_version':mujoco.__version__,
        'description':'Simulated native-camera views; uncalibrated against hardware.',
        'fixture':'Unchanged existing open-jaw, 50 g orange-box grasp scenes.',
        'lighting_override':{'headlight_ambient':[.45]*3,'headlight_diffuse':[.6]*3},
        'image_processing':'No crop, rotation, camera override or annotation. Sites hidden.',
        'views':records}
    (destination/'msg_camera_previews.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':render()
