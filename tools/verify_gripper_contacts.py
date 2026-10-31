"""Validate contact coverage and unchanged mechanics against pre-contact packages."""

from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

from build_gripper_contacts import PROFILES, contact_sources
from split_motor_visuals import ROOT, MJCF, URDF
from verify_msg_jaws import canonical


def verify(baseline_mujoco, baseline_urdf):
    report = {'scope':'Contact geometry intentionally changes; no contact-force equivalence is claimed.', 'profiles':{}}
    geometry = json.loads((ROOT/'reports/gripper_contact_geometry.json').read_text())
    for profile, (identifier, sources) in PROFILES.items():
        before = mujoco.MjModel.from_xml_path(str(baseline_mujoco/f'scene_{profile}.xml'))
        after = mujoco.MjModel.from_xml_path(str(MJCF/f'scene_{profile}.xml'))
        arrays = ['body_parentid','body_pos','body_quat','body_ipos','body_iquat','body_mass','body_inertia',
                  'jnt_type','jnt_pos','jnt_axis','jnt_range','dof_damping','dof_armature','dof_frictionloss',
                  'actuator_trnid','actuator_gainprm','actuator_biasprm','actuator_ctrlrange','actuator_forcerange',
                  'eq_data','eq_obj1id','eq_obj2id','cam_pos','cam_quat','key_qpos','key_ctrl']
        for name in arrays:
            np.testing.assert_array_equal(getattr(before,name),getattr(after,name),err_msg=f'{profile}/{name}')
        a,b = mujoco.MjData(before),mujoco.MjData(after)
        rng = np.random.default_rng(2710)
        for _ in range(26):
            q=before.qpos0.copy();q[:6]=rng.uniform(-1.5,1.5,6)
            q[6]=rng.uniform(before.jnt_range[6,0],0);q[7]=-q[6]
            for model,data in [(before,a),(after,b)]:
                data.qpos[:]=q;mujoco.mj_forward(model,data)
            for name in ['xpos','xmat','site_xpos','site_xmat','qM','qfrc_bias']:
                np.testing.assert_array_equal(getattr(a,name),getattr(b,name),err_msg=f'{profile}/{name}')
        old,new = ET.parse(baseline_urdf/f'par6_{profile}.urdf').getroot(),ET.parse(URDF/f'par6_{profile}.urdf').getroot()
        for root in [old,new]:
            for side in ['left','right']:
                link=root.find(f'link[@name="{side}_finger_link"]')
                for collision in list(link.findall('collision')):link.remove(collision)
        assert canonical(old)==canonical(new),f'{profile}: URDF changed outside finger collisions'
        # Original meshes and all visible asset bytes are preserved.
        for baseline,package in [(baseline_mujoco,MJCF),(baseline_urdf,URDF)]:
            for path in baseline.rglob('*'):
                if path.is_file() and path.suffix.lower() in ['.stl','.obj','.dae','.png','.svg','.mtl']:
                    assert path.read_bytes()==(package/path.relative_to(baseline)).read_bytes(),str(path)
        checks={}
        for kind,package,folder in [('mujoco',MJCF,'assets/contacts'),('urdf',URDF,'meshes/contacts')]:
            for index,side in enumerate(['left','right'],1):
                entry=geometry['profiles'][identifier][kind][side]
                source_package,paths=contact_sources(profile,kind,side,index)
                for path in paths:
                    assert hashlib.sha256(path.read_bytes()).hexdigest()==entry['source_sha256'][path.relative_to(source_package).as_posix()]
                source=trimesh.util.concatenate([trimesh.load(path,force='mesh') for path in paths])
                pieces=[trimesh.load(package/folder/identifier/f'{side}_{i:02d}.STL',force='mesh') for i in range(entry['pieces'])]
                covered=np.zeros(len(source.faces),dtype=bool)
                hull=source.convex_hull;n=hull.face_normals;d=np.sum(n*hull.triangles[:,0],axis=1)
                for piece in pieces:
                    assert piece.is_volume and piece.is_convex
                    assert np.max(piece.vertices@n.T-d)<2e-7
                    normals=piece.face_normals;offset=np.sum(normals*piece.triangles[:,0],axis=1)
                    for start in range(0,len(source.faces),1000):
                        triangles=source.triangles[start:start+1000]
                        covered[start:start+1000]|=np.max(triangles@normals.T-offset,axis=(1,2))<2e-7
                assert covered.all(),f'{identifier}/{kind}/{side}: uncovered source triangle'
                checks[f'{kind}/{side}']={'pieces':len(pieces),'source_triangles_covered':int(covered.sum())}
        report['profiles'][identifier]={'unchanged_compiled_arrays':arrays,'poses_checked':26,
            'urdf_only_finger_collisions_changed':True,'visible_assets_byte_identical':True,'geometry':checks}
    (ROOT/'reports/gripper_contact_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Validated all four profiles: inertia, kinematics, visuals, and complete external contact coverage.')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-mujoco',type=Path,required=True)
    parser.add_argument('--baseline-urdf',type=Path,required=True)
    args=parser.parse_args();verify(args.baseline_mujoco,args.baseline_urdf)
