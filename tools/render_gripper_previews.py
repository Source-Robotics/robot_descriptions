"""Refresh standalone-gripper guide previews (needs OpenGL)."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
import mujoco
from PIL import Image
from build_standalone_grippers import GRIPPERS
from model_reference import ROOT


def render(families=None):
    destination=ROOT/'source/grippers/previews'
    destination.mkdir(parents=True,exist_ok=True)
    count=0
    for family in families or GRIPPERS:
        config=GRIPPERS[family]
        for suffix,_,_ in config['profiles']:
            scene=ROOT/f'mujoco/{family}/scene_{suffix}.xml'
            spec=mujoco.MjSpec.from_file(str(scene))
            if family=='msg':
                # Presentation only: lens upright, jaws towards the viewer.
                # Compile a temporary pose; never save it into a runtime model.
                spec.body('flange').quat=[2**-.5,0,-2**-.5,0]
            model=spec.compile()
            model.vis.global_.offheight=520
            model.vis.global_.offwidth=640
            data=mujoco.MjData(model);mujoco.mj_forward(model,data)
            camera=mujoco.MjvCamera()
            camera.lookat[:]=[0,0,.045]
            camera.distance=.30;camera.azimuth=135;camera.elevation=-24
            if family=='msg':
                camera.lookat[:]=[-.035,0,.022]
                camera.distance=.27;camera.azimuth=15;camera.elevation=-18
            options=mujoco.MjvOption()
            options.sitegroup[:]=0
            with mujoco.Renderer(model,height=520,width=640) as renderer:
                renderer.update_scene(data,camera=camera,scene_option=options)
                Image.fromarray(renderer.render()).save(destination/f'{family}_{suffix}.png')
            count+=1
    print(f'Rendered {count} standalone gripper previews.',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gripper',choices=GRIPPERS,help='Refresh only this gripper')
    args=parser.parse_args()
    render([args.gripper] if args.gripper else None)
