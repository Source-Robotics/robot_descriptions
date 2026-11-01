# Moving from the old model folders

[All models](../README.md) | [PAR6](par6.md) | [PAROL6](parol6.md)

The maintained models now live in `Source-Robotics/robot_descriptions`.
The hardware repositories include them through `description/source`.

| Previous location in the hardware repository | Current location |
| --- | --- |
| PAR6: `PAR6 Description/` MJCF files | `description/source/mujoco/par6/scene_*.xml` |
| PAR6: `PAR6 Description/URDF/` | `description/source/urdf/par6_description/par6_*.urdf` |
| PAROL6: `PAROL6_URDF/` | `description/source/urdf/parol6_description/parol6_*.urdf` |
| MSG: `MSG_gripper_description/` | `description/source/mujoco/msg/` or `description/source/urdf/msg_description/` |
| SSG48: `SSG48_gripper_description/` | `description/source/mujoco/ssg48/` or `description/source/urdf/ssg48_description/` |
| PAROL6: native MuJoCo models | `description/source/mujoco/parol6/scene_*.xml` |

Initialize an existing hardware checkout with:

```bash
git submodule update --init --recursive
```

Update your loader paths and keep the package assets together. Use the robot
guides to choose the equivalent end effector, and check the
[public name migration table](../urdf/par6_description/MIGRATION.md) for code
that refers to older joint or link names. PAROL6 now uses the shared PAR6 public
axis directions and J6 orientation; its corrected geometry is documented in the
[kinematics comparison](../reports/par6_parol6_kinematics.md).

The old folders are removed from the current robot branches to avoid maintaining
two model sources. Their history remains available:

- [Previous PAR6 model folder](https://github.com/Source-Robotics/PAR6-Collaborative-Robot-Arm/tree/c73c11bf820af7dd450312be7c154c957ed4aa26/PAR6%20Description)
- [Previous PAROL6 model folder](https://github.com/Source-Robotics/PAROL6-Desktop-robot-arm/tree/1e7fa6391440660a5c4ce2166e8a33e7e3d334cf/PAROL6_URDF)

The shared release now also provides standalone MSG and SSG48 packages. Each
contains both current jaw options, with inherited tool-relative definitions.
Their older exports are replaced by those packages and remain accessible in history:

- [Previous MSG standalone folder](https://github.com/Source-Robotics/MSG-compliant-AI-stepper-gripper/tree/704e557111701975dab7fa7ca32851c4c161cf2d/MSG_gripper_description)
- [Previous SSG48 standalone folder](https://github.com/Source-Robotics/SSG-48-adaptive-electric-gripper/tree/df776202ba9045acf792f4d5cdceda8d71072c81/SSG48_gripper_description)

The initial shared integrations used `models/descriptions`. The current layout
uses **`description/README.md`** as the product-specific entry and nests the Git
submodule at **`description/source`**. After pulling that move in an existing
checkout, run:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

If Git leaves the old initialized `models/descriptions` directory behind,
check it for local work before removing that untracked copy. A fresh recursive
clone uses the current layout directly. This layout change does not alter model
geometry or the robot's kinematics/dynamics.
