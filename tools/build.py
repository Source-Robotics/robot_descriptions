"""Build and validate both robot families from this checkout alone."""

import argparse
from pathlib import Path
import sys

# Also support Python's isolated (-I) mode and a caller outside this checkout.
sys.path.insert(0,str(Path(__file__).resolve().parent))


def build(models_only=False, render=False):
    import json
    import build_side_badge as badge
    from model_colors import load, badge_config, apply_package
    from package_layout import documentation
    palette = load()  # Reject misspelled/invalid settings before changing models.
    if not models_only:
        import split_motor_visuals as motors
        import build_msg_jaws as msg
        import build_ssg48_jaws as ssg
        from build_gripper_contacts import build as contacts
        print('Rebuilding visual partitions, SVG texture, CAD jaws and contacts.',flush=True)
        motors.generate_meshes();motors.update_urdf();motors.update_mjcf()
        config=json.loads(msg.CONFIG.read_text());report=msg.build_meshes(config);msg.update_models(config)
        (motors.ROOT/'reports/grippers/msg_jaw_geometry.json').write_text(json.dumps(report,indent=2)+'\n')
        config=json.loads(ssg.CONFIG.read_text());report=ssg.build_meshes(config);ssg.update_models(config)
        (motors.ROOT/'reports/grippers/ssg48_jaw_geometry.json').write_text(json.dumps(report,indent=2)+'\n')
        contacts()
    # A color-only edit also updates the baked cover, without retessellating CAD.
    config=badge_config(json.loads(badge.CONFIG.read_text()), palette)
    report=badge.build_meshes(config);badge.update_models(config)
    (badge.ROOT/'reports/appearance/side_badge_geometry.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    from sync_model_dynamics import synchronize
    from build_grasp_benches import build as benches
    from verify_dynamics_sync import verify
    from verify_msg_jaws import verify as msg_checks
    from verify_ssg48_jaws import verify as ssg_checks
    from audit_rcb_dynamics import audit
    from package_models import build as packages
    from model_reference import ROOT
    synchronize()
    from build_pneumatic import build as pneumatic
    pneumatic()
    for package in [ROOT/'mujoco/par6',ROOT/'urdf/par6_description']:
        apply_package(package, 'par6', palette)
    benches();verify()
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):msg_checks();ssg_checks()
    print('MSG and SSG48 mesh, coupling and closing checks passed.',flush=True)
    audit()
    for package in [ROOT/'mujoco/par6',ROOT/'urdf/par6_description']:
        (documentation(package)/'DYNAMICS_AUDIT.md').write_bytes((ROOT/'reports/dynamics/rcb_model_comparison.md').read_bytes())
    from build_parol6 import build as parol6
    from verify_parol6 import verify as parol6_checks
    parol6();parol6_checks()
    from verify_ssg48_contacts import verify as internal_contacts
    from grasp_check import PROFILES as grasp_profiles, run_case
    import json
    internal_contacts()
    grasps=[run_case(profile,shape,robot=robot)
            for robot in ['par6','parol6'] for profile in grasp_profiles
            for shape in ['box','cylinder']]
    assert all(result['passed'] for result in grasps),grasps
    (ROOT/'reports/grippers/grasp_checks.json').write_text(json.dumps(grasps,indent=2)+'\n',encoding='utf-8')
    for package in [ROOT/'mujoco/par6',ROOT/'mujoco/parol6',
                    ROOT/'urdf/par6_description',ROOT/'urdf/parol6_description']:
        (documentation(package)/'SSG48_CONTACTS.md').write_bytes((ROOT/'reports/grippers/ssg48_contact_diagnosis.md').read_bytes())
        (package/'LICENSE').write_bytes((ROOT/'LICENSE').read_bytes())
    print('All 16 electric-gripper close/lift/hold/release checks passed.',flush=True)
    from build_review_sheets import build as reviews
    reviews(refresh=render)
    from build_standalone_grippers import build as grippers
    from verify_standalone_grippers import verify as gripper_checks
    grippers();gripper_checks()
    from build_raft import build as raft
    from verify_raft import verify as raft_checks
    raft();raft_checks()
    from build_ros_support import build as ros_support
    ros_support()
    from build_moveit_support import build as moveit_support
    moveit_support()
    packages()
    print('Standalone build, dynamics/jaw validation and release archive checks passed.',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models-only',action='store_true',help='Use committed visual/CAD assets; rebuild dynamics, fixtures, validation and ZIPs')
    parser.add_argument('--render',action='store_true',help='Refresh review snapshots with OpenGL after an appearance change')
    args=parser.parse_args();build(args.models_only,args.render)
