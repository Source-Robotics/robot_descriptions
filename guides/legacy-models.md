# Moving from the old model folders

[All models](../README.md) | [PAR6](par6.md) | [PAROL6](parol6.md)

The maintained models now live in `Source-Robotics/robot_descriptions`.
The hardware repositories provide product-specific guide folders and download
links. Runtime model files are available from the central repository or its
release packages.

| Previous location in the hardware repository | Current location in robot_descriptions |
| --- | --- |
| PAR6: `PAR6 Description/` MJCF files | `mujoco/par6/scene_*.xml` |
| PAR6: `PAR6 Description/URDF/` | `urdf/par6_description/par6_*.urdf` |
| PAROL6: `PAROL6_URDF/` | `urdf/parol6_description/parol6_*.urdf` |
| MSG: `MSG_gripper_description/` | `mujoco/msg/` or `urdf/msg_description/` |
| SSG48: `SSG48_gripper_description/` | `mujoco/ssg48/` or `urdf/ssg48_description/` |
| PAROL6: native MuJoCo models | `mujoco/parol6/scene_*.xml` |

Download the package matching your product and format using its guide:
[PAR6](par6.md), [PAROL6](parol6.md), [MSG](msg.md) or [SSG48](ssg48.md).
Keep the whole extracted package together in your working directory.

Update your loader paths and keep the package assets together. Use the robot
guides to choose the equivalent end effector, and check the
[public name migration table](../urdf/par6_description/docs/MIGRATION.md) for code
that refers to older joint or link names. PAROL6 now uses the shared PAR6 public
axis directions and J6 orientation; its corrected geometry is documented in the
[kinematics comparison](../reports/kinematics/par6_parol6_kinematics.md).

The old folders are removed from the current robot branches to avoid maintaining
two model sources. Their history remains available:

- [Previous PAR6 model folder](https://github.com/Source-Robotics/PAR6-Collaborative-Robot-Arm/tree/c73c11bf820af7dd450312be7c154c957ed4aa26/PAR6%20Description)
- [Previous PAROL6 model folder](https://github.com/Source-Robotics/PAROL6-Desktop-robot-arm/tree/1e7fa6391440660a5c4ce2166e8a33e7e3d334cf/PAROL6_URDF)

The shared release now also provides standalone MSG and SSG48 packages. Each
contains both current jaw options, with inherited tool-relative definitions.
Their older exports are replaced by those packages and remain accessible in history:

- [Previous MSG standalone folder](https://github.com/Source-Robotics/MSG-compliant-AI-stepper-gripper/tree/704e557111701975dab7fa7ca32851c4c161cf2d/MSG_gripper_description)
- [Previous SSG48 standalone folder](https://github.com/Source-Robotics/SSG-48-adaptive-electric-gripper/tree/df776202ba9045acf792f4d5cdceda8d71072c81/SSG48_gripper_description)

Earlier integrations used model submodules at `models/descriptions` and later
`description/source`. The hardware repositories now use ordinary guide folders:
`par6_description/`, `parol6_description/`, `msg_description/` and
`ssg48_description/`. These contain README/previews/download links; hardware
checkouts no longer need submodule initialization.

After pulling this change, Git may leave an old initialized submodule directory
on disk as an untracked copy. Check that copy for local work before removing it.
The current tracked guides do not depend on it. The layout change does not alter
the released models, geometry, kinematics or dynamics.
