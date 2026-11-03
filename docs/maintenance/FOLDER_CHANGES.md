# Repository folder changes

[Folder map](../../REPOSITORY_GUIDE.md) · [Documentation](../README.md)

This cleanup groups authoring notes and generated results. Model package paths,
scene/URDF filenames, CLI scripts and ROS launch commands keep their existing
names. Geometry, physical data and joint limits are unchanged.

| Previous location | Current location |
| --- | --- |
| `BUILDING.md` | `docs/maintenance/BUILDING.md` |
| `MODEL_DETAILS.md` | `docs/maintenance/MODEL_DETAILS.md` |
| `RELEASING.md` | `docs/maintenance/RELEASING.md` |
| `output/pdf/robot_kinematics_review.pdf` | `docs/kinematics/robot_kinematics_review.pdf` |
| `releases/v2.12.1.md` | `docs/releases/v2.12.1.md` |
| `releases/v2.13.0.md` | `docs/releases/v2.13.0.md` |
| `releases/v2.14.0.md` | `docs/releases/v2.14.0.md` |
| `releases/v2.15.0.md` | `docs/releases/v2.15.0.md` |
| `releases/v2.15.1.md` | `docs/releases/v2.15.1.md` |
| `releases/v2.16.0.md` | `docs/releases/v2.16.0.md` |
| `releases/v2.16.1.md` | `docs/releases/v2.16.1.md` |
| `releases/v2.17.0.md` | `docs/releases/v2.17.0.md` |
| `releases/v2.18.0.md` | `docs/releases/v2.18.0.md` |
| `reports/dynamics_alignment_plan.md` | `reports/dynamics/dynamics_alignment_plan.md` |
| `reports/dynamics_sync_validation.json` | `reports/dynamics/dynamics_sync_validation.json` |
| `reports/dynamics_sync_validation.md` | `reports/dynamics/dynamics_sync_validation.md` |
| `reports/grasp_checks.json` | `reports/grippers/grasp_checks.json` |
| `reports/grasp_frames.md` | `reports/grippers/grasp_frames.md` |
| `reports/gripper_contact_geometry.json` | `reports/grippers/gripper_contact_geometry.json` |
| `reports/gripper_contact_performance.json` | `reports/grippers/gripper_contact_performance.json` |
| `reports/gripper_contact_performance.md` | `reports/grippers/gripper_contact_performance.md` |
| `reports/gripper_contact_validation.json` | `reports/grippers/gripper_contact_validation.json` |
| `reports/kinematics_comparison.png` | `reports/kinematics/kinematics_comparison.png` |
| `reports/kinematics_comparison.svg` | `reports/kinematics/kinematics_comparison.svg` |
| `reports/kinematics_review/2.10.0_build_validation.md` | `reports/kinematics/review/2.10.0_build_validation.md` |
| `reports/kinematics_review/2.10.0_model_definition_baseline.json` | `reports/kinematics/review/2.10.0_model_definition_baseline.json` |
| `reports/kinematics_review/2.11.0_build_validation.md` | `reports/kinematics/review/2.11.0_build_validation.md` |
| `reports/kinematics_review/2.11.0_model_definition_baseline.json` | `reports/kinematics/review/2.11.0_model_definition_baseline.json` |
| `reports/kinematics_review/README.md` | `reports/kinematics/review/README.md` |
| `reports/kinematics_review/build_validation.md` | `reports/kinematics/review/build_validation.md` |
| `reports/kinematics_review/index.html` | `reports/kinematics/review/index.html` |
| `reports/kinematics_review/model_definition_baseline.json` | `reports/kinematics/review/model_definition_baseline.json` |
| `reports/kinematics_review/validation.json` | `reports/kinematics/review/validation.json` |
| `reports/package_validation.json` | `reports/packages/package_validation.json` |
| `reports/par6_parol6_kinematics.md` | `reports/kinematics/par6_parol6_kinematics.md` |
| `reports/parol6_actuation_smoke.json` | `reports/dynamics/parol6_actuation_smoke.json` |
| `reports/parol6_build.json` | `reports/kinematics/parol6_build.json` |
| `reports/parol6_build_validation.md` | `reports/history/parol6_build_validation.md` |
| `reports/parol6_frame_normalization_validation.md` | `reports/history/parol6_frame_normalization_validation.md` |
| `reports/parol6_models.png` | `docs/images/robots/parol6_models.png` |
| `reports/parol6_validation.json` | `reports/kinematics/parol6_validation.json` |
| `reports/parol6_wrist_mounting.png` | `docs/images/robots/parol6_wrist_mounting.png` |
| `reports/pneumatic_geometry.json` | `reports/grippers/pneumatic_geometry.json` |
| `reports/rcb_model_comparison.json` | `reports/dynamics/rcb_model_comparison.json` |
| `reports/rcb_model_comparison.md` | `reports/dynamics/rcb_model_comparison.md` |
| `reports/ssg48_before_after_validation.json` | `reports/history/ssg48_before_after_validation.json` |
| `reports/ssg48_contact_diagnosis.md` | `reports/grippers/ssg48_contact_diagnosis.md` |
| `reports/ssg48_internal_contacts.json` | `reports/grippers/ssg48_internal_contacts.json` |
| `reports/standalone_build_validation.md` | `reports/history/standalone_build_validation.md` |
| `reports/standalone_grippers_validation.json` | `reports/grippers/standalone_grippers_validation.json` |
| `reports/verification.json` | `reports/history/verification.json` |
| `tools/motor_visual_partitions.json` | `reports/appearance/motor_visual_partitions.json` |
| `tools/motor_visual_validation.json` | `reports/appearance/motor_visual_validation.json` |
| `tools/msg_jaw_geometry.json` | `reports/grippers/msg_jaw_geometry.json` |
| `tools/msg_jaw_performance.json` | `reports/grippers/msg_jaw_performance.json` |
| `tools/msg_jaw_validation.json` | `reports/grippers/msg_jaw_validation.json` |
| `tools/side_badge_geometry.json` | `reports/appearance/side_badge_geometry.json` |
| `tools/side_badge_optimization_performance.json` | `reports/appearance/side_badge_optimization_performance.json` |
| `tools/side_badge_performance.json` | `reports/appearance/side_badge_performance.json` |
| `tools/ssg48_jaw_geometry.json` | `reports/grippers/ssg48_jaw_geometry.json` |
| `tools/ssg48_jaw_validation.json` | `reports/grippers/ssg48_jaw_validation.json` |
| `tools/visual_performance.json` | `reports/appearance/visual_performance.json` |

Older release tags and published ZIPs keep their original layouts. Use the
current folder map when browsing `main` or a newly downloaded checkout.
