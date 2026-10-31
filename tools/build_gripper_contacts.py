"""Build a small union of convex finger collision pieces, preserving all inertia.

VHACD proposes up to six regions. Each source triangle is assigned to one
region, whose hull is refitted through the original triangle vertices. This
keeps every external-contact source triangle inside a collision piece and
avoids voxelization shrinking the contact envelope. SSG48 URDF visuals also
include internal transmission racks; use the paired rack-free native export
for contacts, since the joint coupling already represents that mechanism.
No visible meshes or inertial definitions are changed.
Dependencies: numpy, scipy, trimesh, vhacdx. These are build dependencies only.
"""

from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from split_motor_visuals import ROOT, MJCF, URDF
from build_msg_jaws import stl_bytes

PROFILES={
    'msg':('msg_solid',{'mujoco':['assets/jaw{index}_MSG.STL'],
                        'urdf':['meshes/msg/jaw{index}.STL']}),
    'msg_tpu_glue':('msg_soft_tpu',{'mujoco':['assets/jaws/tpu_glue/{side}_carriage.STL','assets/jaws/tpu_glue/{side}_holder.STL','assets/jaws/tpu_glue/{side}_hex.STL'],
                                 'urdf':['meshes/msg/jaws/tpu_glue/{side}_carriage.STL','meshes/msg/jaws/tpu_glue/{side}_holder.STL','meshes/msg/jaws/tpu_glue/{side}_hex.STL']}),
    'ssg48':('ssg48_solid',{'mujoco':['assets/jaws/ssg48_pinch/{side}.STL'],
                           'urdf':['meshes/ssg48/jaws/pinch/{side}.STL']}),
    'ssg48_stock':('ssg48_fin_ray',{'mujoco':['assets/jaw{index}_SSG48.STL'],
                                 'urdf':['meshes/ssg48/jaw{index}.STL']}),
}


def contact_sources(profile, format_name, side, index):
    """Return the package and meshes defining the external contact envelope."""
    _, sources = PROFILES[profile]
    source_format = ('mujoco' if profile.startswith('ssg48') else format_name)
    package = MJCF if source_format == 'mujoco' else URDF
    return package, [package/value.format(side=side, index=index)
                     for value in sources[source_format]]


def convex_parts(source):
    proposed=[trimesh.Trimesh(**part) for part in trimesh.decomposition.convex_decomposition(
        source,maxConvexHulls=6,resolution=150000,maxRecursionDepth=8,
        maxNumVerticesPerCH=32,asyncACD=False,findBestPlane=True)]
    centers=source.triangles_center
    distances=[]
    for part in proposed:
        planes=part.face_normals
        offsets=np.sum(planes*part.triangles[:,0],axis=1)
        distances.append(np.max(centers@planes.T-offsets,axis=1))
    assigned=np.argmin(distances,axis=0)
    pieces=[]
    for index,part in enumerate(proposed):
        triangles=source.triangles[assigned==index]
        if not len(triangles):continue
        points=triangles.reshape(-1,3)
        # Include nearby original vertices if a region only owns a planar face.
        nearest=np.argsort(np.linalg.norm(source.vertices-points.mean(0),axis=1))[:32]
        hull=trimesh.convex.convex_hull(np.vstack([points,source.vertices[nearest]]))
        if not hull.is_volume or not hull.is_convex:
            # Nearly coplanar CAD vertices occasionally leave a numerical seam
            # in Qhull's default triangulation. Joggling changes triangulation,
            # while the returned vertices remain the original input positions.
            hull=trimesh.convex.convex_hull(np.vstack([points,source.vertices[nearest]]),qhull_options='QJ Qt Pp')
        if not hull.is_volume or not hull.is_convex:raise ValueError('Invalid contact piece')
        n=hull.face_normals;offset=np.sum(n*hull.triangles[:,0],axis=1)
        if np.max(points@n.T-offset)>1e-8:raise ValueError('Contact piece does not enclose its source triangles')
        pieces.append(hull)
    pieces.sort(key=lambda part:tuple(np.round(part.centroid,9)))
    return pieces


def remove_old(root):
    for parent in root.iter():
        for child in list(parent):
            if child.get('name','').startswith('contact_'):parent.remove(child)


def build():
    report_path=ROOT/'reports/gripper_contact_geometry.json'
    previous=json.loads(report_path.read_text()) if report_path.exists() else {}
    report={'method':'VHACD regions refitted through complete source triangles',
            'maximum_pieces_per_finger':6,'visible_geometry_unchanged':True,
            'inertial_definitions_unchanged':True,
            'ssg48_scope':'External carriage and selected jaw; internal URDF racks remain visual only. Joint coupling represents the internal transmission.',
            'profiles':{}}
    for profile,(identifier,sources) in PROFILES.items():
        output={}
        for format_name,package in [('mujoco',MJCF),('urdf',URDF)]:
            extension='xml' if format_name=='mujoco' else 'urdf'
            model_path=package/f'par6_{profile}.{extension}'
            tree=ET.parse(model_path);root=tree.getroot()
            if format_name=='mujoco':remove_old(root)
            directory=package/('assets/contacts' if format_name=='mujoco' else 'meshes/contacts')/identifier
            directory.mkdir(parents=True,exist_ok=True)
            output[format_name]={}
            for index,side in enumerate(['left','right'],1):
                source_package, paths=contact_sources(profile,format_name,side,index)
                mesh=trimesh.util.concatenate([trimesh.load(path,force='mesh') for path in paths])
                hashes={p.relative_to(source_package).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
                cached=previous.get('profiles',{}).get(identifier,{}).get(format_name,{}).get(side,{})
                reuse=(cached.get('source_sha256')==hashes and
                       all((directory/f'{side}_{i:02d}.STL').is_file() for i in range(cached.get('pieces',0))))
                if reuse:
                    pieces=[trimesh.load(directory/f'{side}_{i:02d}.STL',force='mesh') for i in range(cached['pieces'])]
                elif format_name=='urdf' and profile.startswith('ssg48'):
                    # Both formats share the same external geometry and frame.
                    # Reuse the native pieces instead of independently refitting
                    # hulls across the long internal racks in the URDF visual.
                    native=MJCF/'assets/contacts'/identifier
                    pieces=[trimesh.load(native/f'{side}_{i:02d}.STL',force='mesh')
                            for i in range(output['mujoco'][side]['pieces'])]
                else:
                    pieces=convex_parts(mesh)
                if format_name=='mujoco':
                    body=root.find(f'.//body[@name="{side}_finger_link"]')
                    visual=body.find('geom')
                    params={key:value for key,value in visual.attrib.items()
                            if key in ['friction','solref','solimp','condim','priority','margin','gap','solmix']}
                    visual.set('contype','0');visual.set('conaffinity','0');visual.set('mass','0')
                    for number,piece in enumerate(pieces):
                        name=f'contact_{identifier}_{side}_{number:02d}'
                        path=directory/f'{side}_{number:02d}.STL'
                        if not reuse:path.write_bytes(stl_bytes(piece,'PAR6 convex finger contact piece; metres'))
                        ET.SubElement(root.find('asset'),'mesh',name=name,file=path.relative_to(package/'assets').as_posix())
                        ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,rgba='0 0 0 0',
                                      group='3',mass='0',**params)
                else:
                    link=root.find(f'link[@name="{side}_finger_link"]')
                    for collision in list(link.findall('collision')):link.remove(collision)
                    for number,piece in enumerate(pieces):
                        path=directory/f'{side}_{number:02d}.STL'
                        if not reuse:path.write_bytes(stl_bytes(piece,'PAR6 convex finger contact piece; metres'))
                        collision=ET.SubElement(link,'collision',name=f'contact_{identifier}_{side}_{number:02d}')
                        ET.SubElement(collision,'origin',xyz='0 0 0',rpy='0 0 0')
                        geometry=ET.SubElement(collision,'geometry')
                        ET.SubElement(geometry,'mesh',filename=path.relative_to(package).as_posix())
                # All pieces use source vertices, so the union stays inside the
                # old single convex hull, while retaining the complete surface.
                hull=mesh.convex_hull
                rng=np.random.default_rng(2710)
                probes=rng.uniform(mesh.bounds[0],mesh.bounds[1],(12000,3))
                def inside(part):
                    n=part.face_normals;d=np.sum(n*part.triangles[:,0],axis=1)
                    return np.max(probes@n.T-d,axis=1)<=1e-9
                old_inside=inside(hull)
                new_inside=np.any([inside(part) for part in pieces],axis=0)
                reclaimed=int(np.count_nonzero(old_inside & ~new_inside))
                output[format_name][side]={'pieces':len(pieces),'triangles':sum(len(p.faces) for p in pieces),
                    'source_package':source_package.relative_to(ROOT).as_posix(),
                    'source_sha256':hashes,
                    'all_source_triangles_enclosed':True,
                    'sampled_old_hull_volume_removed_fraction':reclaimed/max(1,int(old_inside.sum()))}
            ET.indent(tree,space='  ')
            tree.write(model_path,encoding='utf-8',xml_declaration=True)
        report['profiles'][identifier]=output
        print(identifier,{kind:{side:entry['pieces'] for side,entry in value.items()} for kind,value in output.items()},flush=True)
    (ROOT/'reports/gripper_contact_geometry.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':build()
