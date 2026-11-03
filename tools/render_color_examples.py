# SPDX-License-Identifier: Apache-2.0
"""Render color-guide comparisons without changing runtime models (needs OpenGL)."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mujoco
import numpy as np
from PIL import Image, ImageDraw

import build_side_badge as badge
from model_colors import ROOT, CONFIG, apply_tree, badge_config, load, validate

DESTINATION = ROOT/'docs/images/color_examples'
W, H, CAPTION = 640, 480, 42
EXAMPLES = {
    'motors': {'colors': {'structure': '#CAD1EE', 'motors': '#2060E0',
                           'camera_lens': '#080808', 'msg_soft_inserts': '#FFD21A'}},
    'jaws': {'colors': {'msg_soft_holders': '#26364A', 'msg_soft_inserts': '#20B7A8',
                         'ssg48_solid_jaws': '#F08020'}},
    'badge': {'overrides': {'par6_panel': '#303A48'},
              'badge': {'background': '#15364A', 'emblem': '#20B7A8',
                        'symbol': '#15364A', 'wordmark': '#20B7A8',
                        'robot_name': '#FFFFFF', 'tagline': '#FFFFFF', 'divider': '#20B7A8'}},
}


def palette_for(base, changes):
    result = copy.deepcopy(base)
    for section, values in changes.items():
        result[section].update(values)
    return validate(result)


def badge_texture(directory, palette):
    """Bake into the task's temporary directory, preserving the checked cache."""
    original = badge.CONFIG
    config = badge_config(json.loads(original.read_text()), palette)
    directory.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(original.parent/config['source'], directory/config['source'])
    try:
        badge.CONFIG = directory/original.name
        shapes, _ = badge.artwork(config)
        viewport = json.loads((ROOT/'tools/side_badge_geometry.json').read_text())['texture_viewbox_svg_units']
        return badge.render_texture(config, shapes, viewport)
    finally:
        badge.CONFIG = original


def render_tile(directory, family, suffix, profile, palette, view, texture=None):
    package = ROOT/f'mujoco/{family}'
    path = package/f'{family}_{suffix}.xml'
    tree = ET.parse(path)
    apply_tree(tree.getroot(), family, profile, palette)
    compiler = tree.find('compiler')
    for key in ['meshdir', 'texturedir']:
        compiler.set(key, str(package/compiler.get(key, '')))
    if texture is not None:
        texture_path = directory/'badge.png'; texture_path.write_bytes(texture)
        tree.find('asset/texture[@name="par6_badge_texture"]').set('file', str(texture_path))
    model_path = directory/'model.xml'
    tree.write(model_path, encoding='utf-8', xml_declaration=True)
    scene = ET.parse(package/f'scene_{suffix}.xml')
    scene.find('include').set('file', str(model_path))
    scene_path = directory/'scene.xml'
    scene.write(scene_path, encoding='utf-8', xml_declaration=True)
    spec = mujoco.MjSpec.from_file(str(scene_path))
    if family == 'msg':
        spec.body('flange').quat = [2**-.5, 0, -2**-.5, 0]  # Presentation pose only.
    model = spec.compile()
    model.vis.global_.offwidth = W; model.vis.global_.offheight = H
    data = mujoco.MjData(model)
    if view == 'arm':
        mujoco.mj_resetDataKeyframe(model, data, 1)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    if view == 'arm':
        camera.lookat[:] = [0, 0, .20]
        camera.distance = .68; camera.azimuth = 135; camera.elevation = -18
    elif view == 'badge':
        body = data.body('upper_arm_link')
        camera.lookat[:] = body.xpos+body.xmat.reshape(3, 3)@np.array([0., -.105, .0817])
        camera.distance = .55; camera.azimuth = 270; camera.elevation = 0
        model.vis.global_.orthographic = 1; model.vis.global_.fovy = .245
    elif family == 'msg':
        camera.lookat[:] = [-.035, 0, .022]
        camera.distance = .27; camera.azimuth = 15; camera.elevation = -18
    else:
        camera.lookat[:] = [0, 0, .045]
        camera.distance = .30; camera.azimuth = 135; camera.elevation = -24
    options = mujoco.MjvOption(); options.sitegroup[:] = 0; options.geomgroup[3] = 0
    with mujoco.Renderer(model, width=W, height=H) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=options)
        picture = Image.fromarray(renderer.render())
    recipe = {'source_model': path.relative_to(ROOT).as_posix(),
              'model_sha256': hashlib.sha256(path.read_bytes().replace(b'\r\n', b'\n')).hexdigest(),
              'palette': palette, 'qpos': data.qpos.tolist(),
              'presentation_root_rotation': family == 'msg',
              'camera': {'lookat': camera.lookat.tolist(), 'distance': float(camera.distance),
                         'azimuth': float(camera.azimuth), 'elevation': float(camera.elevation),
                         'orthographic': bool(model.vis.global_.orthographic)}}
    return picture, recipe


def render():
    before = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in [
        CONFIG, badge.CONFIG, badge.CONFIG.parent/'texture_cache/par6_badge.png',
        *ROOT.glob('mujoco/*/*.xml'), *ROOT.glob('urdf/*/*.urdf')]}
    base = load(); DESTINATION.mkdir(parents=True, exist_ok=True)
    manifest = {'mujoco_version': mujoco.__version__, 'examples': {},
                'description': 'Native model renders; only example palettes and documented viewing poses differ.'}
    scratch = ROOT/'dist'; scratch.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='color_examples_', dir=scratch) as temporary:
        directory = Path(temporary).resolve(); assert directory.is_relative_to(scratch.resolve())
        for example, rows in [('motors', [('parol6', 'msg_tpu_glue', 'msg_tpu_glue', 'arm', 'PAROL6 / MSG')]),
                              ('jaws', [('msg', 'soft_tpu', 'msg_tpu_glue', 'gripper', 'MSG / Soft TPU jaws'),
                                        ('ssg48', 'solid', 'ssg48', 'gripper', 'SSG48 / Solid jaws')]),
                              ('badge', [('par6', 'flange', 'flange', 'badge', 'PAR6 / side-panel badge')])]:
            output = Image.new('RGB', (W*2, (H+CAPTION)*len(rows)), '#f3f6fa')
            records = []
            for row, (family, suffix, profile, view, label) in enumerate(rows):
                for column, palette in enumerate([base, palette_for(base, EXAMPLES[example])]):
                    tile_dir = directory/f'{example}_{row}_{column}'; tile_dir.mkdir()
                    texture = badge_texture(tile_dir/'artwork', palette) if example == 'badge' and column else None
                    tile, record = render_tile(tile_dir, family, suffix, profile, palette, view, texture)
                    x, y = column*W, row*(H+CAPTION)
                    output.paste(tile, (x, y))
                    caption = label+' / '+('Starting colors' if column == 0 else 'Example colors')
                    ImageDraw.Draw(output).text((x+14, y+H+10), caption, fill='#183247', font_size=18)
                    records.append(record)
            path = DESTINATION/f'{example}.png'; output.save(path, optimize=True)
            manifest['examples'][example] = {'settings_to_change': EXAMPLES[example], 'views': records,
                'image': path.name, 'image_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
            print(f'Rendered {path.relative_to(ROOT)}', flush=True)
    assert all(hashlib.sha256(path.read_bytes()).hexdigest() == digest for path, digest in before.items())
    manifest['runtime_models_palette_and_checked_texture_unchanged'] = True
    (DESTINATION/'render_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    render()
