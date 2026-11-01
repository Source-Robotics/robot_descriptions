"""Refresh the four committed standalone-gripper preview images (needs OpenGL)."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
import mujoco
from PIL import Image
from build_standalone_grippers import GRIPPERS
from model_reference import ROOT


def render():
    destination=ROOT/'source/grippers/previews'
    destination.mkdir(parents=True,exist_ok=True)
    for family,config in GRIPPERS.items():
        for suffix,_,_ in config['profiles']:
            model=mujoco.MjModel.from_xml_path(str(ROOT/f'mujoco/{family}/scene_{suffix}.xml'))
            model.vis.global_.offheight=520
            model.vis.global_.offwidth=640
            data=mujoco.MjData(model);mujoco.mj_forward(model,data)
            camera=mujoco.MjvCamera()
            camera.lookat[:]=[0,0,.045]
            camera.distance=.30;camera.azimuth=135;camera.elevation=-24
            options=mujoco.MjvOption()
            options.sitegroup[:]=0
            with mujoco.Renderer(model,height=520,width=640) as renderer:
                renderer.update_scene(data,camera=camera,scene_option=options)
                Image.fromarray(renderer.render()).save(destination/f'{family}_{suffix}.png')
    print('Rendered four standalone gripper previews.',flush=True)


if __name__=='__main__':render()
