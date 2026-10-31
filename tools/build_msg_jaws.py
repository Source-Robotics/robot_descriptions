"""Build the selectable MSG TPU honeycomb jaw package from the supplied STEP.

Run from the description root with numpy, scipy, trimesh, and gmsh installed:
    python tools/build_msg_jaws.py

The stock MSG entry is never edited. Rebuild after editing the palette or
placement in source/jaws/msg_tpu_glue/jaw.json. Gmsh is a build dependency only.
The selected jaws use new convex collision hulls and the existing finger inertia.
"""

from pathlib import Path
import hashlib
import io
import json
import re
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from split_motor_visuals import ROOT, URDF, MJCF, read_parts

CONFIG = ROOT / 'source/jaws/msg_tpu_glue/jaw.json'
PROFILE = 'msg_tpu_glue'
SOURCE_JAWS = {
    'left': ('jaw1_MSG.STL', 'jaw1.STL', '9230322709aab12df0e039d88e7b77a3f0e0e8b6f2c4e274e359e1066abdd4de'),
    'right': ('jaw2_MSG.STL', 'jaw2.STL', '2c0d7206ba05af4228f8c3a4486af4a77dc8aa7beafa73514a8c0e09a119f4b1'),
}


def rgba(hex_color):
    if not re.fullmatch(r'#[0-9a-fA-F]{6}', hex_color):
        raise ValueError(f'Expected #RRGGBB, got {hex_color}')
    return ' '.join(f'{int(hex_color[i:i+2],16)/255:.9g}' for i in [1,3,5])+' 1'


def stl_bytes(mesh, description='PAR6 MSG TPU honeycomb jaw; metres'):
    result = bytearray(mesh.export(file_type='stl'))
    result[:80] = description.encode('ascii')[:80].ljust(80,b' ')
    return bytes(result)


def step_solids(config, config_path=CONFIG):
    import gmsh
    source = config_path.parent / config['source']
    if hashlib.sha256(source.read_bytes()).hexdigest() != config['source_sha256']:
        raise ValueError('STEP changed; inspect solid IDs and mounting transform before rebuilding')
    gmsh.initialize()
    try:
        gmsh.option.setNumber('General.Terminal',0)
        gmsh.option.setNumber('General.NumThreads',1)
        gmsh.model.occ.importShapes(str(source))
        gmsh.model.occ.synchronize()
        solids = gmsh.model.getEntities(3)
        if {tag for _,tag in solids} != set(config['solid_ids'].values()):
            raise ValueError(f'Expected STEP solid IDs {config["solid_ids"]}, got {solids}')
        gmsh.option.setNumber('Mesh.MeshSizeMax',config['mesh_size_max_mm'])
        gmsh.option.setNumber('Mesh.MeshSizeMin',.15)
        gmsh.option.setNumber('Mesh.MeshSizeFromCurvature',config['curvature_segments'])
        gmsh.option.setNumber('Mesh.MeshSizeFromPoints',0)
        gmsh.option.setNumber('Mesh.MeshSizeExtendFromBoundary',0)
        gmsh.option.setNumber('Mesh.RandomSeed',1)
        gmsh.model.mesh.generate(2)
        tags,coordinates,_ = gmsh.model.mesh.getNodes()
        vertices = np.asarray(coordinates).reshape(-1,3)
        indices = {int(tag):i for i,tag in enumerate(tags)}
        result,report = {},{}
        for name,tag in config['solid_ids'].items():
            faces = []
            for _,surface in gmsh.model.getBoundary([(3,tag)],oriented=False):
                types,_,nodes = gmsh.model.mesh.getElements(2,surface)
                for element_type,element_nodes in zip(types,nodes):
                    if element_type != 2:
                        raise ValueError('Expected first-order triangle surface elements')
                    faces.extend([[indices[int(n)] for n in row] for row in np.asarray(element_nodes).reshape(-1,3)])
            mesh = trimesh.Trimesh(vertices=vertices.copy(),faces=faces,process=True)
            mesh.remove_unreferenced_vertices()
            mesh.fix_normals()
            if not mesh.is_volume:
                raise ValueError(f'{name} is not a closed, outward-oriented solid')
            exact_volume = gmsh.model.occ.getMass(3,tag)
            error = abs(mesh.volume/exact_volume-1)
            if error > .002:
                raise ValueError(f'{name} tessellation volume error {error:.3%}')
            result[name] = mesh
            report[name] = {'step_solid':tag,'triangles':len(mesh.faces),
                            'cad_volume_mm3':exact_volume,'mesh_volume_mm3':mesh.volume,
                            'relative_volume_error':error,'watertight':True}
        return result,report,gmsh.__version__
    finally:
        gmsh.finalize()


def carrier_records(package,side):
    mj_name,urdf_name,expected = SOURCE_JAWS[side]
    path = package/'assets'/mj_name if package==MJCF else package/'meshes/msg'/urdf_name
    blob,records,groups = read_parts(path)
    # The carriage is the separate mounting block extending to X=16.4 mm.
    matches = [g for g in groups if abs(records['vertex'][g,:,0].max()-.0164)<1e-7]
    if len(groups)!=3 or len(matches)!=1:
        raise ValueError(f'Unexpected stock jaw components: {path}')
    if package==MJCF and hashlib.sha256(blob).hexdigest()!=expected:
        raise ValueError(f'Stock jaw changed: {path}')
    ids = np.sort(matches[0])
    return blob[:80]+len(ids).to_bytes(4,'little')+records[ids].tobytes()


def build_meshes(config):
    solids,report,gmsh_version = step_solids(config)
    transform = np.eye(4)
    transform[:3,:3] = config['source_to_left_finger_rotation']
    transform[:3,3] = config['source_to_left_finger_translation_m']
    fitted = {}
    for side in ['left','right']:
        fitted[side] = {}
        for name,source in solids.items():
            mesh = source.copy()
            mesh.apply_scale(.001)  # STEP millimetres to native model metres.
            # A reflection creates the opposing handed jaw; trimesh reverses
            # winding when applying a transform with negative determinant.
            mesh.apply_transform(transform)
            if side=='right':
                mesh.apply_transform(np.diag([1.,-1.,1.,1.]))
            if not mesh.is_volume:
                raise ValueError('Jaw transform reversed the solid orientation')
            fitted[side][name] = mesh
    outputs = {}
    for package,directory in [(MJCF,MJCF/'assets/jaws/tpu_glue'),(URDF,URDF/'meshes/msg/jaws/tpu_glue')]:
        directory.mkdir(parents=True,exist_ok=True)
        for side in ['left','right']:
            carrier = carrier_records(package,side)
            (directory/f'{side}_carriage.STL').write_bytes(carrier)
            carrier_mesh = trimesh.load(file_obj=io.BytesIO(carrier),file_type='stl')
            all_parts = [carrier_mesh]
            for name,mesh in fitted[side].items():
                (directory/f'{side}_{name}.STL').write_bytes(stl_bytes(mesh))
                all_parts.append(mesh)
            hull = trimesh.util.concatenate(all_parts).convex_hull
            (directory/f'{side}_collision.STL').write_bytes(stl_bytes(hull))
            outputs[side] = {'bounds_m':hull.bounds.tolist(),'collision_triangles':len(hull.faces),
                             'carriage_triangles':len(carrier_mesh.faces),
                             'holder_triangles':len(fitted[side]['holder'].faces),
                             'yellow_hex_triangles':len(fitted[side]['hex'].faces)}
        (directory/'jaw.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf-8')
    return {'source_sha256':config['source_sha256'],'gmsh_version':gmsh_version,
            'source_units':'mm','model_units':'m','source_solids':report,'jaws':outputs,
            'collision':'one convex hull of the selected geometry per finger',
            'inertia':'existing finger inertial definitions retained',
            'new_bodies':0,'new_joints':0,'colors':config['colors']}


def update_models(config):
    # Generate a separate entry so users can always return to the stock jaws.
    tree = ET.parse(MJCF/'par6_msg.xml')
    root = tree.getroot()
    from jaw_build_common import reset_contacts
    reset_contacts(root)
    root.set('model','PAR6_MSG_TPU_GLUE')
    assets = root.find('asset')
    for key,color in config['colors'].items():
        ET.SubElement(assets,'material',name=f'msg_jaw_{key}',rgba=rgba(color),specular='0.2',shininess='0.2')
    for side in ['left','right']:
        body = root.find(f'.//body[@name="{side}_finger_link"]')
        collision = body.find('geom')
        collision.set('name',f'{side}_jaw_collision')
        collision.set('rgba','0 0 0 0')
        collision.set('group','3')
        assets.find(f'mesh[@name="{side}_finger_link"]').set('file',f'jaws/tpu_glue/{side}_collision.STL')
        for part in ['carriage','holder','hex']:
            name = f'msg_tpu_{side}_{part}'
            ET.SubElement(assets,'mesh',name=name,file=f'jaws/tpu_glue/{side}_{part}.STL')
            ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,
                          material='msg_jaw_'+('yellow' if part=='hex' else 'black'),
                          contype='0',conaffinity='0',mass='0',group='2')
    ET.indent(tree,space='  ')
    tree.write(MJCF/f'par6_{PROFILE}.xml',encoding='utf-8',xml_declaration=True)
    scene = (MJCF/'scene_msg.xml').read_text(encoding='utf-8')
    scene = scene.replace('PAR6 MSG scene','PAR6 MSG TPU honeycomb jaw scene').replace('par6_msg.xml',f'par6_{PROFILE}.xml')
    (MJCF/f'scene_{PROFILE}.xml').write_text(scene,encoding='utf-8',newline='\n')

    tree = ET.parse(URDF/'par6_msg.urdf')
    root = tree.getroot()
    reset_contacts(root, {side:f'meshes/msg/jaws/tpu_glue/{side}_collision.STL' for side in ['left','right']})
    root.set('name','par6_msg_tpu_glue')
    for side in ['left','right']:
        link = root.find(f'link[@name="{side}_finger_link"]')
        for visual in list(link.findall('visual')):
            link.remove(visual)
        link.find('collision/geometry/mesh').set('filename',f'meshes/msg/jaws/tpu_glue/{side}_collision.STL')
        for part in ['carriage','holder','hex']:
            visual = ET.SubElement(link,'visual',name=f'msg_tpu_{side}_{part}')
            ET.SubElement(visual,'origin',xyz='0 0 0',rpy='0 0 0')
            geometry = ET.SubElement(visual,'geometry')
            ET.SubElement(geometry,'mesh',filename=f'meshes/msg/jaws/tpu_glue/{side}_{part}.STL')
            color = 'yellow' if part=='hex' else 'black'
            material = ET.SubElement(visual,'material',name=f'msg_jaw_{color}')
            ET.SubElement(material,'color',rgba=rgba(config['colors'][color]))
    ET.indent(tree,space='  ')
    tree.write(URDF/f'par6_{PROFILE}.urdf',encoding='utf-8',xml_declaration=True)


if __name__=='__main__':
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    report = build_meshes(config)
    update_models(config)
    from jaw_build_common import refresh_contacts
    refresh_contacts()
    (ROOT/'tools/msg_jaw_geometry.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
