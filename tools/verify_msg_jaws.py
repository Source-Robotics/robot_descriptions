"""Verify the optional jaw meshes, URDF definitions, and retained MJCF physics.

Uses numpy, scipy, trimesh, and mujoco. Stock MSG models are the reference;
only the selected finger visuals and collision geometry may differ.
"""

from pathlib import Path
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

from split_motor_visuals import ROOT, MJCF, URDF


def canonical(element):
    return (element.tag,tuple(sorted(element.attrib.items())),(element.text or '').strip(),
            tuple(canonical(child) for child in element))


def verify():
    stock = mujoco.MjModel.from_xml_path(str(MJCF/'scene_msg.xml'))
    selected = mujoco.MjModel.from_xml_path(str(MJCF/'scene_msg_tpu_glue.xml'))
    arrays = ['body_parentid','body_pos','body_quat','body_mass','body_inertia','body_ipos','body_iquat',
              'jnt_type','jnt_pos','jnt_axis','jnt_range','jnt_limited',
              'dof_damping','dof_armature','dof_frictionloss','qpos0',
              'actuator_trnid','actuator_gainprm','actuator_biasprm','actuator_ctrlrange',
              'actuator_forcerange','eq_data','eq_obj1id','eq_obj2id',
              'cam_pos','cam_quat','cam_fovy','key_qpos','key_ctrl']
    for name in arrays:
        np.testing.assert_array_equal(getattr(stock,name),getattr(selected,name),err_msg=name)
    old,new = ET.parse(URDF/'par6_msg.urdf').getroot(),ET.parse(URDF/'par6_msg_tpu_glue.urdf').getroot()
    new.set('name',old.get('name'))
    for tree in [old,new]:
        for side in ['left','right']:
            link = tree.find(f'link[@name="{side}_finger_link"]')
            for child in list(link):
                if child.tag in ['visual','collision']:
                    link.remove(child)
    assert canonical(old)==canonical(new),'Non-jaw URDF definitions changed'
    a,b = mujoco.MjData(stock),mujoco.MjData(selected)
    rng=np.random.default_rng(2710)
    for _ in range(26):
        pose=np.r_[rng.uniform(-1,1,6),rng.uniform(-.0525,0),0.]
        pose[-1]=-pose[-2]
        for model,data in [(stock,a),(selected,b)]:
            data.qpos[:]=pose
            mujoco.mj_forward(model,data)
        np.testing.assert_array_equal(a.xpos,b.xpos)
        np.testing.assert_array_equal(a.xmat,b.xmat)
        np.testing.assert_array_equal(a.qM,b.qM)
        np.testing.assert_array_equal(a.qfrc_bias,b.qfrc_bias)
    # Run the selected plant through a closing command to catch invalid hulls
    # or an unstable mechanism that static pose comparisons would miss.
    mujoco.mj_resetDataKeyframe(selected,b,0)
    b.ctrl[selected.actuator('gripper_actuator').id]=-.0525
    for _ in range(1000):
        mujoco.mj_step(selected,b)
        assert np.all(np.isfinite(b.qpos)) and np.all(np.isfinite(b.qvel))
    left=selected.joint('left_finger_joint').qposadr[0]
    right=selected.joint('right_finger_joint').qposadr[0]
    coupling_error=float(abs(b.qpos[left]+b.qpos[right]))
    assert coupling_error<1e-4
    mesh_checks={}
    for directory in [MJCF/'assets/jaws/tpu_glue',URDF/'meshes/msg/jaws/tpu_glue']:
        for side in ['left','right']:
            parts=[trimesh.load(directory/f'{side}_{part}.STL',force='mesh')
                   for part in ['carriage','holder','hex']]
            hull=trimesh.load(directory/f'{side}_collision.STL',force='mesh')
            assert hull.is_volume and hull.is_convex
            points=np.vstack([mesh.vertices for mesh in parts])
            # Half-space containment includes boundary vertices without relying
            # on ray casting exactly along coincident hull edges.
            planes=hull.face_normals
            offsets=np.sum(planes*hull.triangles[:,0],axis=1)
            assert np.max(points@planes.T-offsets)<2e-7
            for mesh in parts:
                assert mesh.is_volume
            mesh_checks[f'{directory.relative_to(ROOT)}/{side}']={
                'closed_colored_parts':True,'collision_hull_contains_all_visuals':True,
                'visual_triangles':sum(len(x.faces) for x in parts),
                'collision_triangles':len(hull.faces)}
    report={'mujoco_version':mujoco.__version__,'poses_checked':26,
            'jaw_closing_simulation_steps':1000,'closing_coupling_error_m':coupling_error,
            'unchanged_compiled_arrays':arrays,'urdf_non_jaw_definitions_unchanged':True,
            'kinematics_and_mass_matrices_identical':True,
            'collision_geoms_before':int(np.count_nonzero((stock.geom_contype!=0)|(stock.geom_conaffinity!=0))),
            'collision_geoms_after':int(np.count_nonzero((selected.geom_contype!=0)|(selected.geom_conaffinity!=0))),
            'meshes':mesh_checks,
            'physical_scope':'Current finger mass and inertia retained by request. Collision hulls follow the new rigid jaw geometry; contact forces can differ.'}
    (ROOT/'reports/grippers/msg_jaw_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    verify()
