"""Make solid pinch fingers the default SSG48 jaws, with the old set selectable.

The supplied STEP matches the existing light pinch-finger export. Verify that
match, then combine its exact triangle records with each original carriage.
Run with numpy, scipy, trimesh, rtree, and gmsh installed. Gmsh is build-only.
"""

from pathlib import Path
import hashlib
import io
import json
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from split_motor_visuals import ROOT, MJCF, URDF, read_parts
from build_msg_jaws import rgba, step_solids

CONFIG=ROOT/'source/jaws/ssg48_pinch/jaw.json'


def records_for(path,carriage):
    blob,records,groups=read_parts(path)
    candidates=[g for g in groups if (records['vertex'][g,:,2].max()<1e-6)==carriage]
    if len(groups)!=2 or len(candidates)!=1:
        raise ValueError(f'Unexpected SSG48 carriage/finger components: {path}')
    return blob,records[np.sort(candidates[0])]


def record_mesh(records):
    return trimesh.Trimesh(vertices=records['vertex'].reshape(-1,3),
                           faces=np.arange(len(records)*3).reshape(-1,3),process=True)


def build_meshes(config):
    solids,cad_report,gmsh_version=step_solids(config,CONFIG)
    source=solids['finger']
    source.apply_scale(.001)
    transform=np.eye(4)
    transform[:3,:3]=config['source_to_left_finger_rotation']
    transform[:3,3]=config['source_to_left_finger_translation_m']
    source.apply_transform(transform)
    report={'source_sha256':config['source_sha256'],'gmsh_version':gmsh_version,
            'cad':cad_report,'default':'pinch','new_bodies':0,'new_joints':0,
            'new_geoms':0,'color':config['color'],'jaws':{},
            'inertia':config['inertia'],'collision':config['collision']}
    for index,side in enumerate(['left','right'],1):
        candidate=ROOT/config['matching_export']/f'jaw{index}.STL'
        if hashlib.sha256(candidate.read_bytes()).hexdigest()!=config['matching_export_sha256'][side]:
            raise ValueError(f'Pinch export changed: {candidate}')
        _,finger=records_for(candidate,False)
        fitted=source.copy()
        if side=='right':fitted.apply_transform(np.diag([1.,-1.,1.,1.]))
        light=record_mesh(finger)
        # Both directions check the entire part, not just mounting landmarks.
        _,a,_=trimesh.proximity.closest_point(fitted,light.vertices)
        _,b,_=trimesh.proximity.closest_point(light,fitted.vertices)
        deviation=float(max(a.max(),b.max())*1000)
        if deviation>config['maximum_mesh_deviation_mm']:
            raise ValueError(f'Pinch export does not match the STEP: {deviation:.4f} mm')
        entry={'finger_triangles':len(finger),'max_step_mesh_surface_deviation_mm':deviation,'packages':{}}
        for package,directory,original in [
            (MJCF,MJCF/'assets/jaws/ssg48_pinch',MJCF/'assets'/f'jaw{index}_SSG48.STL'),
            (URDF,URDF/'meshes/ssg48/jaws/pinch',URDF/'meshes/ssg48'/f'jaw{index}.STL'),
        ]:
            header,carriage=records_for(original,True)
            combined=np.concatenate([carriage,finger])
            data=header[:80]+len(combined).to_bytes(4,'little')+combined.tobytes()
            mesh=trimesh.load(io.BytesIO(data),file_type='stl')
            if not mesh.is_volume:
                raise ValueError('Combined SSG48 jaw is not a closed compound mesh')
            directory.mkdir(parents=True,exist_ok=True)
            (directory/f'{side}.STL').write_bytes(data)
            (directory/'jaw.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf-8')
            entry['packages']['mujoco' if package==MJCF else 'urdf']={
                'carriage_triangles':len(carriage),'total_triangles':len(combined),
                'bounds_mm':(mesh.bounds*1000).tolist()}
        report['jaws'][side]=entry
    return report


def preserve_stock():
    for package,extension in [(MJCF,'xml'),(URDF,'urdf')]:
        stock=package/f'par6_ssg48_stock.{extension}'
        if not stock.exists():
            original=package/f'par6_ssg48.{extension}'
            if 'ssg48_pinch' in original.read_text(encoding='utf-8'):
                raise ValueError('Original SSG48 model missing; refusing to overwrite the stock option')
            shutil.copyfile(original,stock)
    # The stock definition is the editable base for regenerating the default.
    scene=(MJCF/'scene_ssg48.xml').read_text(encoding='utf-8')
    scene=scene.replace('par6_ssg48.xml','par6_ssg48_stock.xml')
    (MJCF/'scene_ssg48_stock.xml').write_text(scene,encoding='utf-8',newline='\n')


def update_models(config):
    preserve_stock()
    tree=ET.parse(MJCF/'par6_ssg48_stock.xml')
    root=tree.getroot()
    from jaw_build_common import reset_contacts
    reset_contacts(root)
    root.set('model','PAR6_SSG48_PINCH')
    asset=root.find('asset')
    ET.SubElement(asset,'material',name='ssg48_jaw_material',rgba=rgba(config['color']),
                  specular='0.2',shininess='0.2')
    for side in ['left','right']:
        asset.find(f'mesh[@name="{side}_finger_link"]').set('file',f'jaws/ssg48_pinch/{side}.STL')
        geom=root.find(f'.//body[@name="{side}_finger_link"]/geom')
        geom.attrib.pop('rgba',None)
        geom.set('material','ssg48_jaw_material')
    ET.indent(tree,space='  ')
    tree.write(MJCF/'par6_ssg48.xml',encoding='utf-8',xml_declaration=True)

    tree=ET.parse(URDF/'par6_ssg48_stock.urdf')
    root=tree.getroot()
    reset_contacts(root, {side:f'meshes/ssg48/jaws/pinch/{side}.STL' for side in ['left','right']})
    root.set('name','par6_ssg48_pinch')
    for side in ['left','right']:
        link=root.find(f'link[@name="{side}_finger_link"]')
        for mesh in link.findall('.//mesh'):
            mesh.set('filename',f'meshes/ssg48/jaws/pinch/{side}.STL')
        material=link.find('visual/material')
        material.set('name','ssg48_jaw_material')
        material.find('color').set('rgba',rgba(config['color']))
    ET.indent(tree,space='  ')
    tree.write(URDF/'par6_ssg48.urdf',encoding='utf-8',xml_declaration=True)


if __name__=='__main__':
    config=json.loads(CONFIG.read_text(encoding='utf-8'))
    report=build_meshes(config)
    update_models(config)
    from jaw_build_common import refresh_contacts
    refresh_contacts()
    (ROOT/'reports/grippers/ssg48_jaw_geometry.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
