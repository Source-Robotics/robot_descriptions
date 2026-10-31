"""Bake the supplied Figma SVG onto the existing upper-arm cover visual.

Edit source/artwork/par6_side_badge.json and rerun to update both distributions.
Dependencies: numpy, scipy, mujoco, shapely, trimesh, cairosvg, Pillow,
and pycollada. Colors remain editable in the JSON; rerun to bake them into PNG.
The source SVG is preserved. Tiny SVG blur/stroke effects are below 0.02 mm at
the selected scale; the model uses the underlying flat vector fills.
"""

from pathlib import Path
from datetime import datetime
import hashlib
import io
import json
import re
import xml.etree.ElementTree as ET

import numpy as np
import mujoco
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import polygonize, unary_union
import trimesh
import cairosvg
import collada
from PIL import Image

from split_motor_visuals import ROOT, URDF, MJCF, read_parts

CONFIG = ROOT / 'source/artwork/par6_side_badge.json'
PARTS = ('background', 'emblem', 'symbol', 'wordmark', 'robot_name', 'tagline', 'divider')
BADGE_NAMES = set()  # The existing upper_arm_structure_visual carries the print.
BADGE_MATERIALS = {'par6_badge_material'}
BADGE_TEXTURES = {'par6_badge_texture'}
LEGACY_NAMES = {f'par6_badge_{part}_visual' for part in PARTS} | {'par6_badge_visual'}
LEGACY_MATERIALS = {f'par6_badge_{part}' for part in PARTS}
SVG_NS = {'s': 'http://www.w3.org/2000/svg'}


def cubic_points(start, control1, control2, end, tolerance, depth=0):
    chord = end - start
    length = np.linalg.norm(chord)
    if length < 1e-12:
        error = max(np.linalg.norm(control1-start), np.linalg.norm(control2-start))
    else:
        direction = chord / length
        def distance(point):
            t = np.clip(np.dot(point-start, direction), 0, length)
            return np.linalg.norm(point - start - t*direction)
        error = max(distance(control1), distance(control2))
    if error <= tolerance or depth >= 16:
        return [end]
    a, b, c = (start+control1)/2, (control1+control2)/2, (control2+end)/2
    d, e = (a+b)/2, (b+c)/2
    middle = (d+e)/2
    return (cubic_points(start, a, d, middle, tolerance, depth+1)
            + cubic_points(middle, e, c, end, tolerance, depth+1))


def path_geometry(path, tolerance, evenodd=False):
    tokens = re.findall(r'[A-Za-z]|[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?', path)
    cursor, command, current = 0, None, np.zeros(2)
    contour, contours = [], []
    def finish():
        if len(contour) >= 3:
            ring = np.asarray(contour, dtype=float)
            if not np.array_equal(ring[0], ring[-1]):
                ring = np.vstack([ring, ring[0]])
            contours.append(ring)
        contour.clear()
    while cursor < len(tokens):
        if tokens[cursor].isalpha():
            command = tokens[cursor]
            cursor += 1
        if command == 'Z':
            if contour:
                current = np.asarray(contour[0])
            finish()
            command = None
            continue
        if command not in ['M', 'L', 'H', 'V', 'C']:
            raise ValueError(f'Unsupported SVG path command {command!r}')
        count = {'M': 2, 'L': 2, 'H': 1, 'V': 1, 'C': 6}[command]
        values = np.array([float(t) for t in tokens[cursor:cursor+count]])
        cursor += count
        if command == 'M':
            finish()
            current = values
            contour.append(current.copy())
            command = 'L'
        elif command == 'L':
            current = values
            contour.append(current.copy())
        elif command in ['H', 'V']:
            current = current.copy()
            current[0 if command == 'H' else 1] = values[0]
            contour.append(current.copy())
        else:
            a, b, end = values.reshape(3,2)
            contour.extend(cubic_points(current, a, b, end, tolerance))
            current = end
    finish()
    rings = []
    for ring in contours:
        signed_area = np.sum(ring[:-1,0]*ring[1:,1] - ring[1:,0]*ring[:-1,1]) / 2
        if abs(signed_area) > 1e-8:
            rings.append((Polygon(ring), np.sign(signed_area)))
    lines = unary_union([LineString(ring) for ring in contours])
    filled = []
    for face in polygonize(lines):
        point = face.representative_point()
        winding = sum(sign for ring, sign in rings if ring.covers(point))
        filled_region = int(abs(winding)) % 2 == 1 if evenodd else winding != 0
        if filled_region:
            filled.append(face)
    return unary_union(filled)


def artwork(config):
    source = CONFIG.parent / config['source']
    blob = source.read_bytes()
    expected = config.get('source_sha256')
    if expected and hashlib.sha256(blob).hexdigest() != expected:
        raise ValueError('SVG source changed; inspect its paths before rebuilding this artwork.')
    root = ET.fromstring(blob)
    definitions = {e.get('id'): e for e in root.findall('s:defs/s:path', SVG_NS)}
    tolerance = config['curve_tolerance_svg_units']
    def definition(identifier):
        return path_geometry(definitions[identifier].get('d'), tolerance)
    background_path = root.find('s:path', SVG_NS)
    background = path_geometry(background_path.get('d'), tolerance)
    circle = root.find('s:circle', SVG_NS)
    ellipse = root.find('s:ellipse', SVG_NS)
    background = unary_union([
        background,
        Point(float(circle.get('cx')), float(circle.get('cy'))).buffer(float(circle.get('r')), resolution=128),
        affinity.scale(Point(float(ellipse.get('cx')), float(ellipse.get('cy'))).buffer(1, resolution=128),
                       xfact=float(ellipse.get('rx')), yfact=float(ellipse.get('ry')),
                       origin=(float(ellipse.get('cx')), float(ellipse.get('cy')))),
    ])
    symbol_paths = []
    for element in root:
        candidates = [element] if element.tag.endswith('path') else list(element) if element.tag.endswith('g') else []
        for e in candidates:
            if e.tag.endswith('path') and e is not background_path and e.get('fill') == '#464545':
                symbol_paths.append(path_geometry(e.get('d'), tolerance, e.get('fill-rule') == 'evenodd'))
    divider = root.find('s:rect', SVG_NS)
    x, y, width, height = (float(divider.get(k)) for k in ['x', 'y', 'width', 'height'])
    return {
        'background': background,
        'emblem': definition('stroke0_1245_33'),
        'symbol': unary_union(symbol_paths),
        'wordmark': definition('stroke2_1245_33'),
        'robot_name': definition('stroke4_1245_33'),
        'tagline': definition('stroke6_1245_33'),
        'divider': box(x, y, x+width, y+height),
    }, blob


def color_rgba(value):
    if not re.fullmatch(r'#[0-9a-fA-F]{6}', value):
        raise ValueError(f'Expected #RRGGBB color, got {value}')
    return ' '.join(f'{int(value[i:i+2],16)/255:.9g}' for i in [1,3,5]) + ' 1'


def texture_cache_key(config, viewport):
    # Cache the canonical rasterization of these authoring inputs. Cairo's
    # antialiasing differs slightly across platforms; polygon iteration order
    # is also not an authoring input. Code/source/palette/atlas edits invalidate
    # the cache, while an unchanged checkout emits byte-identical artwork.
    recipe={'schema':1,'config':config,'viewport':list(viewport),
            'svg_sha256':hashlib.sha256((CONFIG.parent/config['source']).read_bytes()).hexdigest(),
            'generator_sha256':hashlib.sha256(Path(__file__).read_bytes().replace(b'\r\n',b'\n')).hexdigest()}
    return hashlib.sha256(json.dumps(recipe,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def render_texture(config, shapes, viewport):
    width, height = config['texture_width_px'], config['texture_height_px']
    if width < 64 or height < 16 or width > 4096 or height > 4096:
        raise ValueError('Texture dimensions must be between 64x16 and 4096x4096')
    directory=CONFIG.parent/'texture_cache'
    image_path,metadata_path=directory/'par6_badge.png',directory/'par6_badge.json'
    key=texture_cache_key(config,viewport)
    if image_path.is_file() and metadata_path.is_file():
        metadata=json.loads(metadata_path.read_text(encoding='utf-8'))
        blob=image_path.read_bytes()
        if metadata.get('input_sha256')==key and metadata.get('image_sha256')==hashlib.sha256(blob).hexdigest():
            return blob
    left, top, view_width, view_height = viewport
    color_rgba(config['panel_color'])
    elements = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="{left} {top} {view_width} {view_height}" preserveAspectRatio="none">',
                f'<rect x="{left}" y="{top}" width="{view_width}" height="{view_height}" fill="{config["panel_color"]}"/>']
    for part in PARTS:
        color_rgba(config['colors'][part])  # validate colors before adding XML
        shape = shapes[part]
        polygons = [shape] if shape.geom_type == 'Polygon' else list(shape.geoms)
        paths = []
        for poly in polygons:
            for ring in [poly.exterior, *poly.interiors]:
                coords = np.asarray(ring.coords)
                paths.append('M' + 'L'.join(f'{x:.6f},{y:.6f}' for x,y in coords) + 'Z')
        elements.append(f'<path id="{part}" d="{" ".join(paths)}" fill="{config["colors"][part]}" fill-rule="evenodd"/>')
    elements.append('</svg>')
    png = cairosvg.svg2png(bytestring=''.join(elements).encode('utf-8'), output_width=width, output_height=height)
    image = Image.open(io.BytesIO(png)).convert('RGB')
    output = io.BytesIO()
    image.save(output, format='PNG', optimize=True)
    blob=output.getvalue()
    directory.mkdir(parents=True,exist_ok=True)
    image_path.write_bytes(blob)
    metadata_path.write_text(json.dumps({'input_sha256':key,
        'image_sha256':hashlib.sha256(blob).hexdigest()},indent=2)+'\n',encoding='utf-8')
    return blob


def original_normals(path, triangles):
    # Use the original STL compiler's smoothing, including its treatment of
    # coincident vertices. OBJ import otherwise smooths some cover edges differently.
    model = mujoco.MjModel.from_xml_string(
        f'<mujoco><asset><mesh name="cover" file="{path.as_posix()}"/></asset>'
        '<worldbody><geom type="mesh" mesh="cover"/></worldbody></mujoco>')
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.mesh_quat[0])
    rotation = rotation.reshape(3, 3)
    positions = model.mesh_vert @ rotation.T + model.mesh_pos[0]
    if not np.allclose(positions[model.mesh_face], triangles, rtol=0, atol=2e-8):
        raise ValueError('Original STL compiler changed triangle order')
    return model.mesh_normal @ rotation.T, model.mesh_facenormal.copy()


def write_obj(path, mesh, uv, face_uv, normals, face_normals):
    lines = ['# Original upper-arm structure triangles with a UV-mapped side badge.']
    lines += ['v ' + ' '.join(f'{v:.12g}' for v in row) for row in mesh.vertices]
    lines += ['vt ' + ' '.join(f'{v:.12g}' for v in row) for row in uv]
    lines += ['vn ' + ' '.join(f'{v:.12g}' for v in row) for row in normals]
    for face, texcoords, normal_indices in zip(mesh.faces, face_uv, face_normals):
        lines.append('f ' + ' '.join(f'{index+1}/{texcoord+1}/{normal+1}'
                     for index, texcoord, normal in zip(face, texcoords, normal_indices)))
    path.write_text('\n'.join(lines)+'\n', encoding='utf-8')


def write_dae(path, mesh, uv, face_uv, normals, face_normals):
    document = collada.Collada()
    document.assetInfo.created = document.assetInfo.modified = datetime(2026, 10, 27)
    document.assetInfo.unitname, document.assetInfo.unitmeter = 'meter', 1.0
    document.assetInfo.upaxis = 'Z_UP'
    image = collada.material.CImage('badge_image', 'par6_badge.png')
    surface = collada.material.Surface('badge_surface', image)
    sampler = collada.material.Sampler2D('badge_sampler', surface)
    diffuse = collada.material.Map(sampler, 'UVMap')
    effect = collada.material.Effect('badge_effect', [surface, sampler], 'phong',
                                   diffuse=diffuse, specular=(.1,.1,.1,1), shininess=10.0)
    material = collada.material.Material('badge_material', 'PAR6 badge', effect)
    document.images.append(image)
    document.effects.append(effect)
    document.materials.append(material)
    sources = [collada.source.FloatSource('badge_positions', mesh.vertices.copy(), ('X','Y','Z')),
               collada.source.FloatSource('badge_normals', normals.copy(), ('X','Y','Z')),
               collada.source.FloatSource('badge_uv', uv.copy(), ('S','T'))]
    geometry = collada.geometry.Geometry(document, 'badge_geometry', 'PAR6 badge', sources)
    inputs = collada.source.InputList()
    inputs.addInput(0, 'VERTEX', '#badge_positions')
    inputs.addInput(1, 'NORMAL', '#badge_normals')
    inputs.addInput(2, 'TEXCOORD', '#badge_uv', set='0')
    indices = np.column_stack([mesh.faces.reshape(-1), face_normals.reshape(-1), face_uv.reshape(-1)]).reshape(-1)
    geometry.primitives.append(geometry.createTriangleSet(indices, inputs, 'badge_symbol'))
    document.geometries.append(geometry)
    binding = collada.scene.MaterialNode('badge_symbol', material, inputs=[('UVMap','TEXCOORD','0')])
    node = collada.scene.Node('badge_node', children=[collada.scene.GeometryNode(geometry, [binding])])
    scene = collada.scene.Scene('badge_scene', [node])
    document.scenes.append(scene)
    document.scene = scene
    document.write(str(path))
    # pycollada's default seven significant digits can round source float32
    # positions. Retain enough precision for an exact float32 round trip.
    for source in sources:
        source.xmlnode.find('{http://www.collada.org/2005/11/COLLADASchema}float_array').text = (
            ' '.join(f'{value:.12g}' for value in source.data.reshape(-1)))
    document.xmlnode.write(str(path), encoding='utf-8', xml_declaration=True)


def build_meshes(config):
    svg_shapes, source = artwork(config)
    scale = config['scale_m_per_svg_unit']
    # SVG horizontal axis follows the cover from elbow to shoulder. SVG vertical
    # axis points toward positive link X: SVG's image Y axis runs downward,
    # opposite the mathematical plane orientation seen from the cover's +Z side.
    angle = np.radians(config['axis_rotation_deg'])
    a, b, d, e = -np.sin(angle)*scale, np.cos(angle)*scale, np.cos(angle)*scale, np.sin(angle)*scale
    transform = [a, b, d, e, config['x_center_m'] - 927*b/2,
                 config['y_start_m'] - 927*e/2]
    # Remove submicrometre slivers before rasterization, in SVG units.
    svg_shapes = {key: shape.simplify(config['curve_tolerance_svg_units']/10, preserve_topology=True)
                  for key, shape in svg_shapes.items()}
    shapes = {key: affinity.affine_transform(shape, transform) for key, shape in svg_shapes.items()}
    _, records, _ = read_parts(URDF / 'meshes/common/upper_arm.STL')
    vertices = records['vertex']
    face_z = config['panel_face_z_m']
    on_face = (np.max(np.abs(vertices[:,:,2] - face_z), axis=1) < 1e-7) & (records['normal'][:,2] > .9999)
    panel = unary_union([Polygon(triangle[:,:2]) for triangle in vertices[on_face]])
    badge = unary_union(list(shapes.values()))
    outside = badge.difference(panel.buffer(1e-7))
    if outside.area > 1e-10:
        raise ValueError(f'Artwork extends outside the cover flat face by {outside.area*1e6:.3f} mm2')
    # Cover's full flat face defines the image atlas; all other surfaces sample
    # an unprinted corner of that same atlas.
    inverse = np.linalg.inv(np.array([[a,b],[d,e]]))
    panel_svg = (vertices[on_face,:,:2].reshape(-1,2) - np.array(transform[4:])) @ inverse.T
    low = np.floor(panel_svg.min(0) - 24)
    high = np.ceil(panel_svg.max(0) + 24)
    viewport = [*low.tolist(), *(high-low).tolist()]
    texture = render_texture(config, svg_shapes, viewport)
    report = {'source_sha256': hashlib.sha256(source).hexdigest(), 'link': config['link'],
              'panel_face_z_m': face_z, 'flat_face_containment_passed': True,
              'minimum_panel_edge_clearance_m': badge.distance(panel.boundary),
              'bounds_m': list(badge.bounds), 'texture_viewbox_svg_units': viewport, 'covers': {}}
    for package, original, directory, extension in [
        ('mujoco', MJCF / 'assets/visual/upper_arm_structure_visual.STL', MJCF / 'assets/branding', 'obj'),
        ('urdf', URDF / 'meshes/common/visual/upper_arm_structure_visual.STL', URDF / 'meshes/common/branding', 'dae'),
    ]:
        _, source_records, _ = read_parts(original)
        triangles = source_records['vertex']
        positions, indices = np.unique(triangles.reshape(-1,3), axis=0, return_inverse=True)
        mesh = trimesh.Trimesh(vertices=positions, faces=indices.reshape(-1,3), process=False)
        normals, face_normals = original_normals(original, triangles)
        corner_normals = normals[face_normals].reshape(-1, 3)
        # MuJoCo 3.3 uses face normals where an untextured mesh's smoothed
        # normal differs by more than acos(0.8). Textured meshes skip that
        # fallback, so bake the original displayed normals explicitly.
        geometric_normals = np.repeat(mesh.face_normals, 3, axis=0)
        fallback = np.sum(corner_normals * geometric_normals, axis=1) < .8
        corner_normals[fallback] = geometric_normals[fallback]
        front = (np.max(np.abs(triangles[:,:,2]-face_z),axis=1)<1e-7) & (source_records['normal'][:,2]>.9999)
        svg_points = (triangles[:,:,:2].reshape(-1,2)-np.array(transform[4:])) @ inverse.T
        uv = np.tile([.001,.001], (len(svg_points),1))
        corners = np.repeat(front,3)
        uv[corners,0] = (svg_points[corners,0]-low[0])/(high[0]-low[0])
        uv[corners,1] = 1-(svg_points[corners,1]-low[1])/(high[1]-low[1])
        if np.any(uv < 0) or np.any(uv > 1):
            raise ValueError('Cover UV coordinates are outside the image atlas')
        # Align position, normal, and texture indices for portable rendering.
        # Only vertices at a UV seam are duplicated; no triangles are added.
        packed, indices = np.unique(np.column_stack([triangles.reshape(-1,3), uv, corner_normals]),
                                    axis=0, return_inverse=True)
        mesh = trimesh.Trimesh(vertices=packed[:,:3], faces=indices.reshape(-1,3), process=False)
        uv, normals = packed[:,3:5], packed[:,5:]
        face_uv = face_normals = mesh.faces
        directory.mkdir(parents=True, exist_ok=True)
        (directory / 'par6_side_badge.svg').write_bytes(source)
        (directory / 'par6_badge.png').write_bytes(texture)
        path = directory / f'upper_arm_structure_badged.{extension}'
        (write_obj if extension == 'obj' else write_dae)(path, mesh, uv, face_uv, normals, face_normals)
        report['covers'][package] = {'triangles': len(mesh.faces), 'source_triangles_preserved': True,
                                    'source_mesh_sha256': hashlib.sha256(original.read_bytes()).hexdigest()}
    report.update(render_mode='existing_cover_texture', extra_visual_geoms=0, extra_triangles=0,
                  texture_resolution=[config['texture_width_px'],config['texture_height_px']], colors=config['colors'])
    return report


def remove_generated(text):
    # Remove only the badge's own entries; preserve all existing model text.
    names = '|'.join(sorted(BADGE_NAMES | BADGE_MATERIALS | BADGE_TEXTURES | LEGACY_NAMES | LEGACY_MATERIALS))
    text = re.sub(r'^[ \t]*<(?:mesh|material|texture|geom)\b[^>]*\bname="(?:'+names+r')"[^>]*/>\n?', '', text, flags=re.M)
    text = re.sub(r'^[ \t]*<visual\b[^>]*\bname="(?:'+names+r')"[^>]*>.*?</visual>\n?', '', text, flags=re.M|re.S)
    return text


def update_models(config):
    for variant in ['flange','msg','msg_tpu_glue','ssg48','ssg48_stock']:
        path = MJCF / f'par6_{variant}.xml'
        text = remove_generated(path.read_text(encoding='utf-8'))
        assets = ['    <texture name="par6_badge_texture" type="2d" file="assets/branding/par6_badge.png"/>',
                  '    <material name="par6_badge_material" texture="par6_badge_texture" rgba="1 1 1 1"/>',
                  ]
        text = text.replace('  </asset>', '\n'.join(assets) + '\n  </asset>', 1)
        text, count = re.subn(r'(<mesh\b[^>]*name="upper_arm_structure_visual"[^>]*file=")[^"]*(")',
                            r'\g<1>branding/upper_arm_structure_badged.obj\2', text)
        if count != 1:
            raise ValueError('Expected existing upper-arm structure mesh')
        text, count = re.subn(r'<geom\b[^>]*name="upper_arm_structure_visual"[^>]*/>',
                             '<geom name="upper_arm_structure_visual" type="mesh" mesh="upper_arm_structure_visual" material="par6_badge_material" contype="0" conaffinity="0" mass="0" group="2"/>', text)
        if count != 1:
            raise ValueError('Expected existing upper-arm structure geom')
        ET.fromstring(text)
        path.write_text(text, encoding='utf-8', newline='\n')

        path = URDF / f'par6_{variant}.urdf'
        text = remove_generated(path.read_text(encoding='utf-8'))
        visual = '''<visual name="upper_arm_structure_visual">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry><mesh filename="meshes/common/branding/upper_arm_structure_badged.dae" /></geometry>
    </visual>'''
        text, count = re.subn(r'<visual\b[^>]*name="upper_arm_structure_visual"[^>]*>.*?</visual>', visual, text, flags=re.S)
        if count != 1:
            raise ValueError('Expected existing upper-arm structure visual')
        ET.fromstring(text)
        path.write_text(text, encoding='utf-8', newline='\n')


if __name__ == '__main__':
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    report = build_meshes(config)
    update_models(config)
    (ROOT / 'tools/side_badge_geometry.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
