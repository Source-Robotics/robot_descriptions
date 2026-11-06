# SPDX-License-Identifier: Apache-2.0
"""Optional OpenGL refresh of RAFT's two pictures in source/raft/previews (the build copies them into both packages
and docs/images):

  raft.png             the arm with the handle held in front, in the package's viewer scene (1280 x 960)
  raft_isometric.png   the overview picture for the README's "Meet the models" row, in the style of PAR6's and
                       PAROL6's: isometric, slate backdrop, soft shadow (1000 x 800)

Both use the model's palette colours. Run python tools/build.py --models-only afterwards.

    python tools/render_raft_previews.py
"""
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageFilter

ROOT = Path(__file__).resolve().parents[1]
MUJOCO = ROOT/'mujoco/raft'
OUT = ROOT/'source/raft/previews'
W, H = 1280, 960                                  # scene.xml <global offwidth/offheight>


def camera(lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat; cam.distance = distance; cam.azimuth = azimuth; cam.elevation = elevation
    return cam


# The hero (chosen 2026-11-06): the palette's colours (as printed), the handle held in front with the base turned toward
# the camera, J6 rolled +40 deg and the trigger opened wide. -55 deg is past the trigger's -39.63 deg stop: picture only.
HERO_SIZE, HERO_SS = (1000, 800), 2
HERO_POSE = {"base_joint": -28, "shoulder_joint": 24, "elbow_joint": 18, "wrist_1_joint": -20, "wrist_2_joint": 15,
             "wrist_3_joint": 40, "trigger_joint": -55}
HERO_CAMERA = dict(azimuth=215.0, elevation=-30.0)                 # orthographic, as robot_descriptions' heroes
HERO_BACKDROP = ((63, 80, 100), (42, 54, 67))                      # robot_descriptions' hero backdrop: lower left, far corner


def hero(out):
    W_, H_ = HERO_SIZE[0] * HERO_SS, HERO_SIZE[1] * HERO_SS
    xml = f"""<mujoco model="raft hero">
  <include file="raft.xml"/>
  <visual>
    <global offwidth="{W_}" offheight="{H_}" fovy="0.36" orthographic="true"/>
    <quality shadowsize="8192" offsamples="8"/>
    <map shadowclip="1.2" shadowscale="0.8" znear="0.01" zfar="20"/>
    <headlight ambient="0.42 0.42 0.42" diffuse="0.35 0.35 0.35" specular="0.08 0.08 0.08"/>
  </visual>
  <asset><material name="hero_floor" rgba="1 1 1 1" specular="0" shininess="0" reflectance="0"/></asset>
  <worldbody>
    <light name="key" pos="0 0 1.4" dir="0 0 -1" directional="true" castshadow="true" diffuse="0.55 0.55 0.55" specular="0.3 0.3 0.3"/>
    <light pos="0.8 -0.8 0.6" dir="-0.6 0.6 -0.4" directional="true" castshadow="false" diffuse="0.18 0.18 0.2" specular="0.1 0.1 0.1"/>
    <geom name="hero_floor" type="plane" size="4 4 0.01" material="hero_floor" contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>"""
    tmp = MUJOCO / "_hero_scene.xml"                   # next to raft.xml: MuJoCo resolves meshdir from the top file
    tmp.write_text(xml, encoding="utf-8")
    try:
        m = mujoco.MjModel.from_xml_path(str(tmp))
    finally:
        tmp.unlink()
    d = mujoco.MjData(m)
    for name, deg in HERO_POSE.items():
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]] = np.radians(deg)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE; cam.orthographic = 1
    cam.lookat[:] = [0.075, -0.005, 0.105]; cam.distance = 1.2
    cam.azimuth, cam.elevation = HERO_CAMERA["azimuth"], HERO_CAMERA["elevation"]
    a, e = np.radians(cam.azimuth), np.radians(cam.elevation)
    fwd = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    F = np.array([np.cos(a), np.sin(a), 0.0]); R = np.cross(F, [0, 0, 1.0]); up = np.cross(R, fwd)
    # key light from the picture's upper left, a little toward the camera: a short soft shadow, as on PAR6's hero.
    # A directional light's shadow map sits around its position, so the light is placed back along its direction.
    kd = 0.32 * R - 0.12 * F - np.array([0, 0, 1.0]); kd /= np.linalg.norm(kd)
    key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, "key")
    m.light_dir[key] = kd; m.light_pos[key] = np.array([0.08, 0.0, 0.1]) - 1.2 * kd
    mujoco.mj_forward(m, d)
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "hero_floor")
    with mujoco.Renderer(m, H_, W_) as r:
        for _ in range(3):                          # frame the arm: its outline centred, 84 % of the width or 80 % of the height
            r.enable_segmentation_rendering(); r.update_scene(d, camera=cam)
            seg = r.render()[..., 0]; r.disable_segmentation_rendering()
            ys, xs = np.nonzero((seg != floor) & (seg >= 0))
            ppm = H_ / m.vis.global_.fovy           # pixels per metre (orthographic)
            dx, dy = ((xs.min() + xs.max()) / 2 - W_ / 2) / ppm, ((ys.min() + ys.max()) / 2 - H_ / 2) / ppm
            cam.lookat[:] = cam.lookat + dx * R - dy * up
            m.vis.global_.fovy *= max((xs.max() - xs.min()) / (0.84 * W_), (ys.max() - ys.min()) / (0.80 * H_))
        r.enable_segmentation_rendering(); r.update_scene(d, camera=cam)
        seg = r.render()[..., 0]; r.disable_segmentation_rendering()
        r.update_scene(d, camera=cam)
        img = r.render().astype(float)
    is_floor = (seg == floor) | (seg < 0)
    # the backdrop: lightest at the lower left, darker toward the top right; the floor's shading becomes the shadow
    y, x = np.mgrid[0:H_, 0:W_].astype(float) / HERO_SS
    t = np.clip(np.hypot(x, y - 0.75 * HERO_SIZE[1]) / np.hypot(HERO_SIZE[0], 0.75 * HERO_SIZE[1]), 0, 1) ** 0.8
    near, far = (np.array(c, dtype=float) for c in HERO_BACKDROP)
    bg = near * (1 - t[..., None]) + far * t[..., None]
    lum = img.mean(axis=2)
    shade = np.clip(lum / np.percentile(lum[is_floor], 99.5), 0, 1)
    dark = Image.fromarray((255 * np.clip((1 - shade) / 0.3, 0, 1)).astype(np.uint8)).filter(ImageFilter.GaussianBlur(7 * HERO_SS))
    shadow = 1 - 0.38 * np.asarray(dark, dtype=float) / 255
    pic = np.where(is_floor[..., None], bg * shadow[..., None], img)
    Image.fromarray(np.clip(pic, 0, 255).astype(np.uint8)).resize(HERO_SIZE, Image.LANCZOS).save(out / "raft_isometric.png", optimize=True)


def main():
    m = mujoco.MjModel.from_xml_path(str(MUJOCO/'scene.xml')); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, 'handle_in_front'))
    mujoco.mj_forward(m, d)
    OUT.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(m, H, W) as r:
        r.update_scene(d, camera=camera([0.09, 0.0, 0.105], 0.42, 128, -18))
        Image.fromarray(r.render()).save(OUT/'raft.png', optimize=True)
    hero(OUT)
    for p in sorted(OUT.glob('*.png')):
        print(f'  {p.relative_to(ROOT).as_posix()}  {Image.open(p).size}  {p.stat().st_size // 1024} kB')


if __name__ == '__main__':
    main()
