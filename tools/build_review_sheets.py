"""Model-derived, dimensioned kinematic review sheets for every robot/tool.

Normal builds use checked render caches without requiring OpenGL. Refresh the
cache deliberately with --render when geometry, materials or cameras change.
Use --pdf for the optional printable booklet (requires pypdf).
"""

import argparse
import base64
import hashlib
from html import escape
import io
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cairosvg
import mujoco
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from parol6_model import ROOT, PROFILES, ARM_JOINTS, ARM_LINKS, origin, urdf_fk

ROBOTS = ('par6', 'parol6')
LABELS = dict(zip(PROFILES, ('Flange only', 'MSG / Solid jaws',
                           'MSG / Soft TPU jaws', 'SSG48 / Solid jaws',
                           'SSG48 / Fin-ray jaws', 'Pneumatic gripper')))
CACHE = ROOT/'source/reviews/render_cache'
REPORT = ROOT/'reports/kinematics_review'
W, H = 1060, 790
PAGE_HEIGHT = 1697
# Identical orthographic scale and framing on all sheets. At azimuth 270,
# +X points left, +Z up, and the camera sees the robot from its +Y side.
CAMERA = {'lookat': [.11, 0., .192], 'distance': 1., 'azimuth': 270.,
          'elevation': 0., 'world_height_m': .47, 'width': W, 'height': H}
RENDER_REVISION = 2
THETA = ('q1', 'q2 - pi/2', 'q3 + pi', 'q4', 'q5', 'q6 + pi')
ALPHA = np.array([-np.pi/2, np.pi, np.pi/2, -np.pi/2, np.pi/2, np.pi])


def paths(robot, profile):
    return (ROOT/f'urdf/{robot}_description/{robot}_{profile}.urdf',
            ROOT/f'mujoco/{robot}/scene_{profile}.xml')


def dimensions(path):
    """Lengths come from actual URDF joint origins and the fixed tool chain."""
    tree = ET.parse(path)
    xyz = [origin(tree.find(f'joint[@name="{name}"]/origin'))[:3, 3]
           for name in ARM_JOINTS]
    frames = urdf_fk(path)
    tool = np.linalg.inv(frames['wrist_3_link'])@frames['tcp']
    # This six-row specialization assumes a pure X/Z translation after J6.
    if abs(tool[1, 3]) > 1e-10 or Rotation.from_matrix(tool[:3, :3]).magnitude() > 1e-8:
        raise ValueError(f'{path}: TCP needs an additional fixed transform after the six DH rows')
    a = np.array([xyz[1][2], xyz[1][0], np.linalg.norm(xyz[2]),
                  np.linalg.norm(xyz[3]), abs(xyz[4][2]), tool[2, 3], -tool[0, 3]])
    a[np.abs(a) < 1e-14] = 0.
    return a, tool


def dh_frames(q, dims):
    """Standard DH to configured TCP; q are the public package joint angles."""
    a1, a2, a3, a4, a5, a6, a7 = dims
    a = [a2, a3, -a4, 0., 0., -a7]
    d = [a1, 0., 0., -a5, 0., -a6]
    theta = np.asarray(q)+[0., -np.pi/2, np.pi, 0., 0., np.pi]
    result = [np.eye(4)]
    for th, al, length, offset in zip(theta, ALPHA, a, d):
        c, s, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
        step = np.array([[c, -s*ca, s*sa, length*c],
                         [s, c*ca, -c*sa, length*s],
                         [0., sa, ca, offset], [0., 0., 0., 1.]])
        result.append(result[-1]@step)
    return result


def native_pose(data, name, site=False):
    item = data.site(name) if site else data.body(name)
    result = np.eye(4)
    result[:3, 3], result[:3, :3] = item.xpos, item.xmat.reshape(3, 3)
    return result


def error(a, b):
    return np.array([np.linalg.norm(a[:3, 3]-b[:3, 3])*1000,
                     Rotation.from_matrix(a[:3, :3].T@b[:3, :3]).magnitude()*180/np.pi])


def samples(path, count=128):
    tree = ET.parse(path)
    low, high = [], []
    for name in ARM_JOINTS:
        joint = tree.find(f'joint[@name="{name}"]')
        limit = joint.find('limit')
        lo, hi = (-np.pi, np.pi) if joint.get('type') == 'continuous' else (float(limit.get('lower')), float(limit.get('upper')))
        low.append(lo); high.append(hi)
    rng = np.random.default_rng(610310)
    return np.vstack([np.zeros(6), low, high, rng.uniform(low, high, (count-3, 6))])


def analyze(robot, profile):
    upath, mpath = paths(robot, profile)
    dims, tool = dimensions(upath)
    model = mujoco.MjModel.from_xml_path(str(mpath))
    data = mujoco.MjData(model)
    limits = samples(upath)
    maxima = {name: np.zeros(2) for name in ('urdf_mujoco', 'dh_urdf', 'dh_mujoco')}
    for q in limits:
        for name, angle in zip(ARM_JOINTS, q):
            data.qpos[model.joint(name).qposadr] = angle
        mujoco.mj_forward(model, data)
        u = urdf_fk(upath, dict(zip(ARM_JOINTS, q)))['tcp']
        m = native_pose(data, 'tcp', site=True)
        dh = dh_frames(q, dims)[-1]
        for key, left, right in [('urdf_mujoco', u, m), ('dh_urdf', dh, u), ('dh_mujoco', dh, m)]:
            maxima[key] = np.maximum(maxima[key], error(left, right))
    # A documentation build fails if a different axis convention or tool
    # transform no longer fits this table. It must not silently publish it.
    if any(value[0] > .025 or value[1] > .01 for value in maxima.values()):
        raise ValueError(f'{robot}/{profile}: DH/frame discrepancy exceeds review tolerance: {maxima}')
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    centers = {f'J{i}': native_pose(data, link)[:3, 3].tolist()
               for i, link in enumerate(ARM_LINKS, 1)}
    centers['TCP'] = native_pose(data, 'tcp', site=True)[:3, 3].tolist()
    centers['flange'] = native_pose(data, 'flange')[:3, 3].tolist()
    result = {'robot': robot, 'profile': profile, 'label': LABELS[profile],
              'pose_radians': [0.]*6, 'finger_displacements_m': [0., 0.],
              'camera': CAMERA, 'dimension_names': [f'a{i}' for i in range(1, 8)],
              'dimensions_mm': (dims*1000).tolist(),
              'dimension_source': upath.relative_to(ROOT).as_posix(),
              'snapshot_source': mpath.relative_to(ROOT).as_posix(),
              'source_hash_encoding': 'SHA-256 of source text bytes with CRLF normalized to LF',
              'urdf_sha256': hashlib.sha256(upath.read_bytes().replace(b'\r\n', b'\n')).hexdigest(),
              'native_model_sha256': hashlib.sha256((mpath.parent/f'{robot}_{profile}.xml').read_bytes().replace(b'\r\n', b'\n')).hexdigest(),
              'wrist3_to_configured_tcp_m': tool.tolist(),
              'native_zero_pose_centers_m': centers, 'poses_checked': len(limits),
              'max_error_mm_deg': {k: v.tolist() for k, v in maxima.items()},
              'dh_convention': 'Rz(theta) Tz(d) Tx(a) Rx(alpha)',
              'theta': list(THETA), 'alpha_degrees': np.rad2deg(ALPHA).tolist(),
              'a_mm': [dims[1]*1000, dims[2]*1000, -dims[3]*1000, 0., 0., -dims[6]*1000],
              'd_mm': [dims[0]*1000, 0., 0., -dims[4]*1000, 0., -dims[5]*1000]}
    return result, model


def visual_signature(scene):
    digest = hashlib.sha256(json.dumps({'revision': RENDER_REVISION,
        'camera': CAMERA, 'mujoco': mujoco.__version__}, sort_keys=True).encode())
    # Hash canonical source geometry instead of compiled floating-point arrays:
    # mesh principal-axis rounding may differ across operating systems. Ignore
    # explicit inertia/drive data, which cannot change these zero-pose visuals.
    visited = set()
    hidden = {'inertial', 'actuator', 'sensor', 'tendon', 'equality', 'contact',
              'keyframe', 'option', 'position', 'velocity', 'motor', 'general'}
    def visit(path):
        path = path.resolve()
        if path in visited:
            return
        visited.add(path)
        root = ET.parse(path).getroot()
        compiler = root.find('compiler')
        directories = {} if compiler is None else compiler.attrib
        for include in root.findall('.//include'):
            visit(path.parent/include.get('file'))
        for tag, directory in [('mesh', 'meshdir'), ('texture', 'texturedir'), ('hfield', 'assetdir')]:
            for asset in root.findall(f'.//asset/{tag}[@file]'):
                filename = asset.get('file')
                folder = directories.get(directory, directories.get('assetdir', ''))
                source = path.parent/folder/filename
                if not source.is_file():
                    source = path.parent/filename
                blob = source.read_bytes()
                if source.suffix.lower() in {'.obj', '.mtl'}:
                    blob = blob.replace(b'\r\n', b'\n')
                digest.update(filename.encode()); digest.update(blob)
        for parent in root.iter():
            for child in list(parent):
                if child.tag in hidden:
                    parent.remove(child)
            if not (parent.text or '').strip():
                parent.text = None
            parent.tail = None
        digest.update(ET.canonicalize(ET.tostring(root, encoding='unicode')).encode())
    visit(scene)
    return digest.hexdigest()


def project(point, camera):
    pos, forward, up = (np.array(camera[key]) for key in ('pos', 'forward', 'up'))
    right = np.cross(forward, up)
    x, y = (np.asarray(point)-pos)@right, (np.asarray(point)-pos)@up
    height = camera['top']-camera['bottom']
    width = camera['width'] or height*W/H
    return np.array([(x-camera['center']+width/2)/width*W,
                     (camera['top']-y)/height*H])


def render_snapshot(model, path):
    model.vis.global_.offwidth, model.vis.global_.offheight = W, H
    # MuJoCo 3.3 free-camera projection is controlled by visual/global.
    model.vis.global_.orthographic = 1
    model.vis.global_.fovy = CAMERA['world_height_m']
    model.vis.global_.ipd = 0
    data = mujoco.MjData(model)
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = CAMERA['lookat']
    cam.distance, cam.azimuth, cam.elevation = (CAMERA[key] for key in ('distance', 'azimuth', 'elevation'))
    option = mujoco.MjvOption()
    option.sitegroup[:] = 0
    option.geomgroup[3] = 0
    with mujoco.Renderer(model, width=W, height=H) as renderer:
        renderer.update_scene(data, camera=cam, scene_option=option)
        gl = renderer.scene.camera[0]
        assert gl.orthographic == 1, 'Review drawings require orthographic projection'
        projection = {'pos': gl.pos.tolist(), 'forward': gl.forward.tolist(), 'up': gl.up.tolist(),
                      'top': gl.frustum_top, 'bottom': gl.frustum_bottom,
                      'center': gl.frustum_center, 'width': gl.frustum_width,
                      'orthographic': True}
        rgb = renderer.render().copy()
        renderer.enable_segmentation_rendering()
        seg = renderer.render()
        mask = seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)
        floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
        if floor >= 0:
            mask &= seg[:, :, 0] != floor
        rgb[~mask] = 255
        # A clipping check catches future geometry changes outside this common
        # camera. All profiles must retain one comparable drawing scale.
        ys, xs = np.where(mask)
        if len(xs) == 0 or min(xs.min(), ys.min(), W-1-xs.max(), H-1-ys.max()) < 8:
            raise ValueError('Robot is clipped: update the common review camera and rerender all profiles')
        Image.fromarray(rgb).save(path)
        # Independently check the world-to-image registration against actual
        # OpenGL pixels, using three temporary foreground markers. These are
        # rendered only after saving the clean robot snapshot.
        renderer.disable_segmentation_rendering()
        checks = []
        for point in ([0., .65, .05], [.12, .65, .2], [.3, .65, .32]):
            geom = renderer.scene.geoms[renderer.scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                               np.array([.002]*3), np.array(point),
                               np.eye(3).ravel(), np.array([1., 0., 0., 1.], dtype=np.float32))
            renderer.scene.ngeom += 1
            checks.append(project(point, projection))
        marked = renderer.render()
        red = (marked[:, :, 0] > 130) & (marked[:, :, 1] < 90) & (marked[:, :, 2] < 90)
        yy, xx = np.where(red)
        errors = []
        for point in checks:
            nearby = (abs(xx-point[0]) < 10) & (abs(yy-point[1]) < 10)
            if not np.any(nearby):
                raise ValueError('Could not verify dimension-marker projection against rendered pixels')
            actual = np.array([xx[nearby].mean()+.5, yy[nearby].mean()+.5])
            errors.append(float(np.linalg.norm(actual-point)))
        projection['registration_error_max_px'] = max(errors)
        assert max(errors) < 1.5, f'Dimension/image misregistration: {errors}'
    return projection


def cached_snapshot(model, robot, profile, refresh=False):
    CACHE.mkdir(parents=True, exist_ok=True)
    path = CACHE/f'{robot}_{profile}.png'
    meta = path.with_suffix('.json')
    signature = visual_signature(paths(robot, profile)[1])
    if refresh:
        projection = render_snapshot(model, path)
        meta.write_text(json.dumps({'visual_signature': signature, 'projection': projection,
            'image_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}, indent=2)+'\n', encoding='utf-8')
    if not path.is_file() or not meta.is_file():
        raise ValueError('Review snapshot missing. Run python tools/build_review_sheets.py --render')
    details = json.loads(meta.read_text())
    if details['visual_signature'] != signature or details['image_sha256'] != hashlib.sha256(path.read_bytes()).hexdigest():
        raise ValueError(f'{robot}/{profile}: stale review snapshot. Run python tools/build_review_sheets.py --render')
    return path, details['projection']


def number(value):
    value = 0. if abs(value) < 1e-9 else value
    return f'{value:.6f}'.rstrip('0').rstrip('.')


def note(profile):
    if profile == 'pneumatic':
        return ('Pneumatic TCP follows the legacy MHZ2-10D reference; it is not a newly measured grasp center.',
                'Jaws close symmetrically through 3.5 mm each. Mass and finite actuator settings are estimates.')
    if profile == 'flange':
        return ('TCP is the configured flange reference. No gripper is installed.',
                'a6 measures J6 center to flange; a7 is zero.')
    if profile.startswith('msg'):
        return ('MSG jaw variants share this operational TCP; it is not a measured grasp center.',
                'The 28.35 mm lateral TCP offset is included in DH row 6 as a = -a7.')
    return ('SSG48 jaw variants share this operational TCP; it is not a measured grasp center.',
            'Solid jaws are about 23.4 mm shorter than Fin-ray jaws; their TCP was retained.')


def sheet(data, snapshot, projection, page):
    dims = data['dimensions_mm']
    robot, profile = data['robot'], data['profile']
    parts = ['''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="1200" height="1697" viewBox="0 0 1200 1697">
<defs><marker id="arrow" markerWidth="7" markerHeight="7" refX="3.5" refY="3.5" orient="auto-start-reverse"><path d="M0 0L7 3.5L0 7Z" fill="#173c53"/></marker></defs>
<style>text{font-family:Arial,DejaVu Sans,sans-serif;fill:#18364b}.title{font-size:36px;font-weight:700}.subtitle{font-size:23px}.small{font-size:17px}.tiny{font-size:14px}.dim{font-size:21px;font-weight:bold}.joint{font-size:17px;font-weight:bold}.mono{font-family:DejaVu Sans Mono,monospace;font-size:18px}.line{stroke:#173c53;stroke-width:1.5}.ext{stroke:#6a7e8c;stroke-width:1;stroke-dasharray:4 3}.cell{font-size:20px}</style>
<rect width="1200" height="1697" fill="white"/>
<rect x="0" y="0" width="1200" height="10" fill="#16838c"/>
''']
    def text(x, y, value, cls='small', anchor=None, fill=None, rotate=False):
        attrs = (f' text-anchor="{anchor}"' if anchor else '')+(f' style="fill:{fill}"' if fill else '')
        if rotate:
            attrs += f' transform="rotate(-90 {x:.2f} {y:.2f})"'
        if cls in ('dim', 'joint'):
            parts.append(f'<text x="{x:.2f}" y="{y:.2f}" class="{cls}"{attrs} style="fill:white;stroke:white;stroke-width:6;stroke-linejoin:round">{escape(str(value))}</text>')
        parts.append(f'<text x="{x:.2f}" y="{y:.2f}" class="{cls}"{attrs}>{escape(str(value))}</text>')
    def line(x1, y1, x2, y2, cls='line', arrow=False):
        attrs = ' marker-start="url(#arrow)" marker-end="url(#arrow)"' if arrow else ''
        parts.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" class="{cls}"{attrs}/>')
    text(50, 62, f'{robot.upper()}  /  {LABELS[profile]}', 'title')
    text(50, 99, 'Kinematic review  |  Standard DH to the configured TCP', 'subtitle')
    opening = 'no jaws' if profile == 'flange' else 'jaws open'
    text(50, 132, f'Pose: q1 ... q6 = 0 rad; {opening}. Orthographic side view; dimensions in mm.')
    text(1150, 100, f'{page:02d} / {len(ROBOTS)*len(PROFILES)}', 'small', 'end')
    encoded = base64.b64encode(snapshot.read_bytes()).decode()
    ox, oy = 70, 155
    parts.append(f'<image x="{ox}" y="{oy}" width="{W}" height="{H}" xlink:href="data:image/png;base64,{encoded}"/>')
    def xy(point):
        return project(point, projection)+[ox, oy]
    points = {k: xy(v) for k, v in data['native_zero_pose_centers_m'].items()}
    # Model dimensions use the URDF origins. Native pose joint centers provide
    # the image registration; sub-pixel format differences are reported below.
    base, shoulder, elbow, wrist, tip, tcp = [points[k] for k in ('J1', 'J2', 'J3', 'J4', 'J6', 'TCP')]
    def horizontal(p, q, y, label):
        x1, x2 = sorted([p[0], q[0]])
        line(p[0], p[1], p[0], y-8, 'ext'); line(q[0], q[1], q[0], y-8, 'ext')
        line(x1, y, x2, y, arrow=True)
        text((x1+x2)/2, y-12, label, 'dim', 'middle')
    def vertical(p, q, x, label, left=False, rotate=False):
        y1, y2 = sorted([p[1], q[1]])
        line(p[0], p[1], x+(-8 if left else 8), p[1], 'ext')
        line(q[0], q[1], x+(-8 if left else 8), q[1], 'ext')
        line(x, y1, x, y2, arrow=True)
        text(x-12 if left else x+12, (y1+y2)/2+7, label, 'dim', 'middle' if rotate else ('end' if left else 'start'), rotate=rotate)
    horizontal(wrist, tip, 206, f'a5 = {number(dims[4])}')
    horizontal(tip, tcp, 258, f'a6 = {number(dims[5])}')
    vertical(base, shoulder, 1120, f'a1 = {number(dims[0])}', rotate=True)
    vertical(shoulder, elbow, 1020, f'a3 = {number(dims[2])}')
    vertical(elbow, wrist, 1020, f'a4 = {number(dims[3])}')
    horizontal(base, shoulder, 920, 'a2')
    text((base[0]+shoulder[0])/2, 951, number(dims[1]), 'small', 'middle')
    if dims[6] > .001:
        aligned = [tcp[0], tip[1]]
        vertical(aligned, tcp, 190, f'a7 = {number(dims[6])}', left=True)
    else:
        text(107, 375, 'a7 = 0', 'dim')
        text(107, 400, 'No lateral TCP offset', 'small')
    # Indicate the nominal straight centerline through the forearm and tool.
    line(tcp[0]-16, tip[1], wrist[0]+18, tip[1], 'ext')
    for name, dx, dy in [('J1', 18, -10), ('J2', -32, 32), ('J3', 19, 22), ('J4', 20, -17), ('J6', -8, 42)]:
        x, y = points[name]
        parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4.5" fill="#16838c" stroke="white" stroke-width="1.5"/>')
        text(x+dx, y+dy, 'J5 / J6' if name == 'J6' else name, 'joint')
    x, y = tcp
    line(x-8, y, x+8, y); line(x, y-8, x, y+8)
    text(x-17, y+32, 'TCP', 'joint', 'end')
    text(50, 992, 'a1  Base height     a2  Shoulder offset     a3  Upper arm     a4  Elbow offset')
    text(50, 1019, 'a5  Forearm axis distance     a6  J6-to-TCP axial reach     a7  TCP lateral offset')
    text(50, 1055, 'A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)', 'mono')
    text(50, 1081, 'q = model joint angles (rad). a = link length along X; d = link offset along Z.')
    # Correct standard-DH a/d terminology, unlike the swapped headers in the
    # supplied example. Display alpha in degrees and theta in radians.
    xedges = [50, 130, 310, 550, 750, 950, 1150]
    top, rh = 1100, 39
    parts.append(f'<rect x="50" y="{top}" width="1100" height="{rh*7}" fill="#fff" stroke="#c4d1da"/>')
    parts.append(f'<rect x="50" y="{top}" width="1100" height="{rh}" fill="#e5f0f2"/>')
    rows = [['Joint', 'Link', 'theta [rad]', 'alpha [deg]', 'a [mm]', 'd [mm]']]
    names = ('Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3 / TCP')
    a_names, d_names = ('a2', 'a3', '-a4', '', '', '-a7'), ('a1', '', '', '-a5', '', '-a6')
    for i in range(6):
        av, dv = number(data['a_mm'][i]), number(data['d_mm'][i])
        rows.append([str(i+1), names[i], THETA[i], number(data['alpha_degrees'][i]),
                     f'{a_names[i]} = {av}' if a_names[i] else av,
                     f'{d_names[i]} = {dv}' if d_names[i] else dv])
    for ri, row in enumerate(rows):
        if ri > 0 and ri % 2 == 0:
            parts.append(f'<rect x="51" y="{top+rh*ri}" width="1098" height="{rh}" fill="#f5f8fa"/>')
        for ci, value in enumerate(row):
            text((xedges[ci]+xedges[ci+1])/2, top+ri*rh+26, value, 'cell', 'middle')
    for x in xedges[1:-1]:
        line(x, top, x, top+rh*7, 'ext')
    text(50, 1402, note(profile)[0])
    text(50, 1428, note(profile)[1])
    vals = data['max_error_mm_deg']
    text(50, 1468, f'Independent FK checks: {data["poses_checked"]} poses, including zero and joint-limit bounds.', 'small')
    text(50, 1495, f'Max URDF vs MuJoCo TCP: {vals["urdf_mujoco"][0]:.6f} mm / {vals["urdf_mujoco"][1]:.6f} deg', 'small')
    text(50, 1522, f'Max ideal DH vs URDF: {vals["dh_urdf"][0]:.6f} mm / {vals["dh_urdf"][1]:.6f} deg', 'small')
    text(50, 1549, 'Right-angle DH approximates rounded export rotations. The snapshot uses the MuJoCo visual meshes.', 'small')
    line(50, 1570, 1150, 1570, 'ext')
    text(50, 1595, f'Source: {robot}_{profile}.urdf + scene_{profile}.xml  |  Full transforms, hashes and checks: {profile}.json', 'tiny')
    text(50, 1622, 'All sheets use the same scale. Side projection: +X points left, +Z up; view from +Y.', 'small')
    text(50, 1665, 'Source Robotics  /  robot_descriptions  /  Generated from current models; physical-data assumptions: DYNAMICS.md.', 'tiny')
    parts.append('</svg>')
    return '\n'.join(parts)


def markdown(robot, entries):
    lines = [f'# {robot.upper()} kinematic review sheets', '',
        'All six profiles use the same orthographic scale and zero-angle pose. '
        'Rendered meshes come from MuJoCo; dimensions and fixed TCP transforms '
        'come from the corresponding URDF. Each JSON contains source hashes, '
        'the complete fixed transform and independent FK-check results.', '',
        '| Configuration | Diagram | Raster | Numeric data |', '| --- | --- | --- | --- |']
    for item in entries:
        p = item['profile']
        lines.append(f'| {LABELS[p]} | [SVG](kinematics_review/{p}.svg) | [PNG](kinematics_review/{p}.png) | [JSON](kinematics_review/{p}.json) |')
    lines += ['', 'Standard DH: `Rz(theta) Tz(d) Tx(a) Rx(alpha)`. Joint values '
        '`q` use package coordinates; the theta offsets are already in the table. '
        'Apply them once. Alpha is in degrees; theta uses radians. `a` is a '
        'translation along X; `d` is a translation along Z.', '',
        'The last row ends at the **configured operational TCP**. It is not a '
        'measured jaw contact center. Each gripper keeps the same TCP across '
        'its jaw options. In particular, SSG48 Solid jaws are about 23.4 mm '
        'shorter than Fin-ray jaws. See [GRASP_FRAMES.md](GRASP_FRAMES.md).', '',
        'The pneumatic profile retains the legacy operational TCP with '
        '45.25 mm lateral offset and 68.9 / 62.3 mm axial reach on '
        'PAR6 / PAROL6. Its original side-gripping jaws move symmetrically; '
        'mass/actuator values remain estimates. See [PNEUMATIC.md](PNEUMATIC.md).', '',
        'PAR6/PAROL6 share joint-angle conventions and a 177 mm forearm '
        '(`d4 = -177 mm`). The intended flange-table difference is '
        '`d6 = -57.1 / -50.5 mm`, matching the different wrist housings. Small additional '
        'differences in displayed precision are inherited from individual '
        'URDF exports, and the numerical checks measure their effect. '
        'See [KINEMATICS_COMPARISON.md](KINEMATICS_COMPARISON.md).', '',
        'Zero pose is a wrist singularity and is used here only as a clear '
        'dimensioned reference. Joint-center labels are physical joint centers, '
        'not a drawing of every intermediate DH frame. J5 and J6 centers coincide. '
        'The side projection can hide overlapping jaw surfaces.', '',
        'Regenerate in the authoring checkout with '
        '`python tools/build_review_sheets.py`. Add `--render` to refresh '
        'snapshots after visual changes and `--pdf` to assemble the optional booklet.', '']
    return '\n'.join(lines)


def viewer(entries):
    # No server, fetching, or external assets: works directly as file:// HTML.
    encoded = json.dumps([{'id': f'{d["robot"]}_{d["profile"]}',
        'label': f'{d["robot"].upper()} / {d["label"]}',
        'src': f'../../mujoco/{d["robot"]}/kinematics_review/{d["profile"]}.svg'} for d in entries])
    html = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot kinematics comparison</title><style>
*{box-sizing:border-box}body{margin:0;background:#edf2f5;color:#18364b;font:16px system-ui,sans-serif}header{padding:24px 28px;background:white;border-top:7px solid #16838c}h1{margin:0 0 8px;font-size:26px}p{margin:7px 0;max-width:1100px;line-height:1.5}main{display:grid;grid-template-columns:1fr 1fr;gap:18px;padding:20px}section{min-width:0}select{width:100%;font:inherit;padding:12px;background:white;color:inherit;border:1px solid #a8bbc6;border-radius:5px;margin-bottom:12px}img{display:block;width:100%;background:white;box-shadow:0 2px 10px #173c531a}a{color:#086f7a}@media(max-width:800px){main{grid-template-columns:1fr}}.hint{font-size:14px;color:#486173}</style>
<header><h1>PAR6 &amp; PAROL6 · Kinematic review</h1><p>Compare any two robot/tool configurations. Every sheet uses the same pose, orthographic camera and scale. The DH table ends at the configured TCP.</p><p class="hint">Both forearms are 177 mm. PAROL6's shorter wrist places the flange and every tool 6.6 mm closer to J6. Jaw variants retain their tool's operational TCP; this is not a measured contact center. Open a sheet in a new tab to zoom or print.</p></header>
<main><section><select id="left" aria-label="Left model"></select><a id="left-link" target="_blank"><img id="left-img" alt="Left robot diagram and DH table"></a></section><section><select id="right" aria-label="Right model"></select><a id="right-link" target="_blank"><img id="right-img" alt="Right robot diagram and DH table"></a></section></main>
<script>const sheets=__DATA__;for(const [side,index] of [['left',0],['right',__HALF__]]){const select=document.getElementById(side);sheets.forEach((sheet,i)=>{const option=new Option(sheet.label,String(i));select.add(option)});select.value=String(index);const update=()=>{const item=sheets[Number(select.value)];document.getElementById(side+'-img').src=item.src;document.getElementById(side+'-link').href=item.src};select.addEventListener('change',update);update()}</script></html>'''.replace('__DATA__', encoded).replace('__HALF__', str(len(PROFILES)))
    (REPORT/'index.html').write_text(html, encoding='utf-8')


def report_summary(entries):
    maximum = lambda key: np.max([e['max_error_mm_deg'][key] for e in entries], axis=0)
    um, du, dm = (maximum(key) for key in ('urdf_mujoco', 'dh_urdf', 'dh_mujoco'))
    lines = ['# Dimensioned kinematic review', '',
        '[Open the offline two-model comparison](index.html) or '
        '[the twelve-page printable booklet](../../output/pdf/robot_kinematics_review.pdf).', '',
        'Two robots, each with six configurations: flange, MSG Solid jaws, '
        'MSG Soft TPU jaws, SSG48 Solid jaws, SSG48 Fin-ray jaws and the '
        'original side-gripping pneumatic gripper. Each sheet '
        'shows the actual native visual meshes at zero model-joint angles, '
        'using one orthographic camera and scale. No mesh dimensions were '
        'measured from pixels. The dimensions and TCP transforms come from URDF.', '',
        '| Robot / configuration | a1 | a2 | a3 | a4 | a5 | a6 | a7 |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for e in entries:
        lines.append(f'| {e["robot"].upper()} / {e["label"]} | '+' | '.join(number(d) for d in e['dimensions_mm'])+' |')
    lines += ['', 'All lengths above are millimetres. Standard DH uses '
        '`Rz(theta) Tz(d) Tx(a) Rx(alpha)`. The final row includes the configured '
        'TCP. Its theta expressions already contain the required joint offsets; '
        'do not apply those offsets twice.', '',
        '## Numerical checks', '',
        'Each table was evaluated at 128 reproducible poses, including zero '
        'and lower/upper bounds. Revolute joints use their declared limits; '
        'continuous joints use [-pi, pi] for this check. These checks compare '
        'model consistency, not real-hardware calibration.', '',
        '| Comparison | Maximum translation (mm) | Maximum rotation (deg) |',
        '| --- | ---: | ---: |',
        f'| Actual URDF vs native MuJoCo TCP | {um[0]:.6f} | {um[1]:.6f} |',
        f'| Ideal standard DH vs actual URDF TCP | {du[0]:.6f} | {du[1]:.6f} |',
        f'| Ideal standard DH vs native MuJoCo TCP | {dm[0]:.6f} | {dm[1]:.6f} |', '',
        'The right-angle DH table approximates the rounded rotations in some '
        'exports. Shoulder offset is 23.420721 mm in the flange URDF and '
        '23.421 mm in several tool profiles. These precision differences are '
        'retained and their accumulated FK effect is reported above.', '',
        'Snapshot marker registration was checked against three temporary '
        'OpenGL markers for every image; maximum error was '
        f'{max(e["snapshot_registration_error_max_px"] for e in entries):.3f} pixels. '
        'The final snapshots exclude these temporary markers.', '',
        '## Items to review visually', '',
        '- Both robots use a5 = 177 mm (DH d4 = -177 mm). PAROL6 lower-arm '
        'visual/collision placement and the wrist origin move outward 0.65 mm '
        'relative to the historical export.',
        '- PAR6 flange-only a6 = 57.1 mm; PAROL6 a6 = 50.5 mm. Matching '
        'housing-to-tool spacing moves every PAROL6 tool inward 6.6 mm. '
        'The spacing correction is derived from CAD housing faces, not an '
        'independent hardware measurement.',
        '- With either MSG or SSG48, a6 = 140.9 mm on PAR6 and 134.3 mm on '
        'PAROL6. MSG a7 = 28.35 mm; SSG48 and flange-only a7 = 0.',
        '- Pneumatic a6 = 68.9 mm on PAR6 and 62.3 mm on PAROL6, with '
        'a7 = 45.25 mm. Its operational TCP comes from the legacy reference. '
        'Its 195 g total, CAD mass distribution and finite servo settings '
        'are unverified estimates.',
        '- Jaw variants within each gripper share the same operational TCP. '
        'Identical DH tables do not imply identical contact geometry. SSG48 '
        'Solid jaws are about 23.4 mm shorter than Fin-ray jaws.',
        '- These are physical joint-center dimensions, not intermediate DH '
        'frame origins. J5/J6 centers coincide. Zero pose is a wrist '
        'singularity chosen for the drawing; the numerical checks also use '
        'nonzero angles.', '',
        'All four packages contain an independent copy of their six SVG/PNG '
        'sheets plus JSON transforms, source hashes and diagnostics. PAROL6 '
        'forearm geometry and the entire flange subtree use the corrected '
        'placements. Original mesh records, local mass/COM/tensors and finite '
        'drive settings are retained. PAR6 model geometry and physics are unchanged.', '']
    (REPORT/'README.md').write_text('\n'.join(lines), encoding='utf-8')


def build(refresh=False, pdf=False):
    REPORT.mkdir(parents=True, exist_ok=True)
    entries, svgs = [], []
    for robot in ROBOTS:
        family = []
        for profile in PROFILES:
            data, model = analyze(robot, profile)
            image, projection = cached_snapshot(model, robot, profile, refresh)
            data['snapshot_registration_error_max_px'] = projection['registration_error_max_px']
            svg = sheet(data, image, projection, len(entries)+1)
            png = cairosvg.svg2png(bytestring=svg.encode(), output_width=1200, output_height=PAGE_HEIGHT)
            for directory in [ROOT/f'mujoco/{robot}', ROOT/f'urdf/{robot}_description']:
                dest = directory/'kinematics_review'
                dest.mkdir(exist_ok=True)
                (dest/f'{profile}.svg').write_text(svg, encoding='utf-8')
                (dest/f'{profile}.png').write_bytes(png)
                (dest/f'{profile}.json').write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
            family.append(data); entries.append(data); svgs.append(svg)
            print(f'{robot}/{profile}: review sheet and 128-pose DH/TCP checks passed.', flush=True)
        for directory in [ROOT/f'mujoco/{robot}', ROOT/f'urdf/{robot}_description']:
            (directory/'KINEMATIC_REVIEW.md').write_text(markdown(robot, family), encoding='utf-8')
    (REPORT/'validation.json').write_text(json.dumps(entries, indent=2)+'\n', encoding='utf-8')
    viewer(entries)
    report_summary(entries)
    if pdf:
        from pypdf import PdfReader, PdfWriter
        book = PdfWriter()
        for svg in svgs:
            # Explicit A4 portrait page, keeping vector dimensions and table.
            rendered = cairosvg.svg2pdf(bytestring=svg.encode(),
                output_width=595.276*96/72, output_height=841.890*96/72)
            book.add_page(PdfReader(io.BytesIO(rendered)).pages[0])
        book.add_metadata({'/Title': 'PAR6 and PAROL6 - Kinematic review', '/Author': 'Source Robotics',
                           '/Subject': 'Model-derived zero-pose dimensions and standard DH tables for twelve configurations'})
        dest = ROOT/'output/pdf/robot_kinematics_review.pdf'
        dest.parent.mkdir(parents=True, exist_ok=True)
        with dest.open('wb') as output:
            book.write(output)
        print(dest, flush=True)
    return entries


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--render', action='store_true', help='Refresh all orthographic snapshots with OpenGL')
    parser.add_argument('--pdf', action='store_true', help='Also assemble the printable PDF (requires pypdf)')
    args = parser.parse_args()
    build(args.render, args.pdf)
