# Build tools

[Build instructions](../docs/maintenance/BUILDING.md) · [Repository map](../REPOSITORY_GUIDE.md)

Run commands from the repository root. Most changes use only:

```bash
python tools/build.py --models-only
```

The scripts keep their existing paths for compatibility. They are grouped by
purpose below. Generated JSON records now live in [reports/](../reports/README.md).

## Main commands

| Script | Purpose |
| --- | --- |
| [build.py](build.py) | Main build: generates assets, synchronizes physical data, builds all products, validates them and writes ZIPs. |
| [package_models.py](package_models.py) | Creates all eight independent ZIPs, extracts them and checks their contents/assets. |
| [prepare_release.py](prepare_release.py) | Checks version consistency and prepares release assets, manifest and SHA-256 checksums. |
| [export_hardware_guide.py](export_hardware_guide.py) | Copies a product guide and its local previews to a hardware repository, linking model/docs paths to a chosen central revision. |

## Generate models and documentation

| Script | Purpose |
| --- | --- |
| [split_motor_visuals.py](split_motor_visuals.py) | Splits original triangle records into separately colored arm actuators and MSG lens/motor/structure visuals. |
| [build_side_badge.py](build_side_badge.py) | Bakes SVG/palette onto the side-panel visual texture. |
| [build_msg_jaws.py](build_msg_jaws.py) | Tessellates and installs the MSG Soft TPU jaw option. |
| [build_ssg48_jaws.py](build_ssg48_jaws.py) | Tessellates and installs default SSG48 Solid jaws while retaining Fin-ray as an option. |
| [build_gripper_contacts.py](build_gripper_contacts.py) | Generates the convex finger collision pieces while preserving inertia. |
| [build_grasp_benches.py](build_grasp_benches.py) | Creates the box/cylinder electric-gripper test fixtures. |
| [sync_model_dynamics.py](sync_model_dynamics.py) | Applies reference arm mass properties and corrected tool masses to the models. |
| [build_pneumatic.py](build_pneumatic.py) | Splits the original pneumatic assembly into housing/two moving jaws and generates both formats. |
| [build_parol6.py](build_parol6.py) | Generates PAROL6 from included original CAD exports and shared PAR6 tool/physical data. |
| [build_standalone_grippers.py](build_standalone_grippers.py) | Exports independent MSG/SSG48 packages from maintained arm-mounted tool subtrees. |
| [build_kinematics_docs.py](build_kinematics_docs.py) | Writes common-frame/DH comparison and actuation documentation. |
| [build_review_sheets.py](build_review_sheets.py) | Builds twelve dimensioned sheets and their checks; optionally refreshes OpenGL snapshots or generates the PDF. |
| [build_ros_support.py](build_ros_support.py) | Generate native ROS 2 packaging/launch support and retain ROS 1 entry points. |
| [build_moveit_support.py](build_moveit_support.py) | Generate separate MoveIt 2 configuration packages from maintained models. |

## Validate and compare

| Script | Purpose |
| --- | --- |
| [verify_motor_visuals.py](verify_motor_visuals.py) | Checks visual partition preservation and unchanged mechanics against supplied earlier packages. |
| [verify_msg_jaws.py](verify_msg_jaws.py) | Checks MSG jaw geometry, model definitions, coupling and retained physics. |
| [verify_ssg48_jaws.py](verify_ssg48_jaws.py) | Checks default Solid jaws and the retained Fin-ray option. |
| [verify_gripper_contacts.py](verify_gripper_contacts.py) | Compares contact coverage and mechanics with supplied pre-contact packages. |
| [verify_dynamics_sync.py](verify_dynamics_sync.py) | Checks applied reference dynamics and measured masses; can also compare against a supplied older package. |
| [audit_rcb_dynamics.py](audit_rcb_dynamics.py) | Compares reference and model physical properties in consistent frames. |
| [verify_parol6.py](verify_parol6.py) | Checks PAROL6 frames, source-chain correspondence, shared physical data and tool transforms. |
| [verify_ssg48_contacts.py](verify_ssg48_contacts.py) | Sweeps both jaw options for unintended SSG48 internal contacts on both arms. |
| [verify_standalone_grippers.py](verify_standalone_grippers.py) | Compares standalone and mounted grippers, runs close/open cycles and checks relocated packages. |
| [grasp_check.py](grasp_check.py) | Simulates close, lift, hold and release in those fixtures. |

## Render and benchmark

| Script | Purpose |
| --- | --- |
| [render_color_examples.py](render_color_examples.py) | Renders the color guide's motor, jaw and badge comparisons from temporary model copies; needs OpenGL. |
| [render_gripper_previews.py](render_gripper_previews.py) | Optional OpenGL refresh of standalone guide images. MSG uses a temporary upright presentation pose, without changing saved mounting frames. |
| [render_msg_camera_previews.py](render_msg_camera_previews.py) | Renders the actual named MSG camera in both existing box grasp fixtures, with recorded demo lighting and PNG/camera metadata. |
| [render_parol6_previews.py](render_parol6_previews.py) | Optional OpenGL refresh of PAROL6 preview images. |
| [benchmark_visuals.py](benchmark_visuals.py) | Local timing comparison for rendering, physics and clearance queries. Timing records are measurements on a particular machine. |

## Shared helpers and reference import

| Script | Purpose |
| --- | --- |
| [model_reference.py](model_reference.py) | Portable NumPy DH/dynamics implementation and repository paths, used instead of requiring the live RCB runtime. |
| [parol6_model.py](parol6_model.py) | Coordinate/frame helpers for the original PAROL6 chain and shared tools. |
| [model_colors.py](model_colors.py) | Validates the shared palette, resolves overrides and applies visual colors without changing physical definitions. |
| [model_licensing.py](model_licensing.py) | Ships CC BY 4.0 model attribution and Apache-2.0 support-code notices in each package. |
| [package_layout.py](package_layout.py) | Shared docs/previews paths, documentation index and package-local link checks. |
| [jaw_build_common.py](jaw_build_common.py) | Shared jaw-generation helpers. |
| [ros_urdf.py](ros_urdf.py) | Converts relative URDF mesh references to ROS package URIs. |
| [import_rcb_reference.py](import_rcb_reference.py) | Explicitly imports a fresh snapshot from a specified RCB checkout. This optional operation needs that external checkout; the normal build does not. |

## ROS support templates

- [ros_support/](ros_support/): installed URDF helpers, display launch template and Linux display checks.
- [moveit_support/](moveit_support/): planning/mock-control templates and Linux integration checks.

These are support-code sources. Users launch the installed packages under
`urdf/` or `moveit/`; see [ROS 2](../docs/ROS2.md) and [MoveIt](../docs/MOVEIT2.md).
