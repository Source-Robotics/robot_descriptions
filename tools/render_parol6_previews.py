"""Optional OpenGL preview rendering; not required for a headless model build."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco
from PIL import Image, ImageDraw

from parol6_model import ROOT, MJCF, URDF, PROFILES


from package_layout import previews

def render():
    labels = ['Flange only', 'MSG / Solid jaws', 'MSG / Soft TPU jaws',
              'SSG48 / Solid jaws', 'SSG48 / Fin-ray jaws', 'Pneumatic gripper']
    tile_width, tile_height = 480, 500
    montage = Image.new('RGB', (tile_width*3, tile_height*2), '#f3f6fa')
    for index, (profile, label) in enumerate(zip(PROFILES, labels)):
        model = mujoco.MjModel.from_xml_path(str(MJCF/f'scene_{profile}.xml'))
        model.vis.global_.offwidth = tile_width
        model.vis.global_.offheight = tile_height-40
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 1)
        mujoco.mj_forward(model, data)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0., 0., .20]
        camera.distance = .85
        camera.azimuth = 135
        camera.elevation = -18
        with mujoco.Renderer(model, height=tile_height-40, width=tile_width) as renderer:
            renderer.update_scene(data, camera=camera)
            tile = Image.fromarray(renderer.render())
        tile.save(previews(MJCF)/f'parol6_{profile}.png')
        tile.save(previews(URDF)/f'parol6_{profile}.png')
        x, y = (index % 3)*tile_width, (index//3)*tile_height
        montage.paste(tile, (x, y))
        ImageDraw.Draw(montage).text((x+14, y+tile_height-30), label, fill='#183247', font_size=20)
    path = ROOT/'reports/parol6_models.png'
    montage.save(path)
    print(path)


if __name__ == '__main__':
    render()
