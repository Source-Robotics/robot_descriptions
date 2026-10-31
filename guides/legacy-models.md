# Moving from the old robot model folders

[All models](../README.md) | [PAR6](par6.md) | [PAROL6](parol6.md)

The maintained models now live in `Source-Robotics/robot_descriptions`.
The hardware repositories include them through `models/descriptions`.

| Previous location in the hardware repository | Current location |
| --- | --- |
| PAR6: `PAR6 Description/` MJCF files | `models/descriptions/mujoco/par6/scene_*.xml` |
| PAR6: `PAR6 Description/URDF/` | `models/descriptions/urdf/par6_description/par6_*.urdf` |
| PAROL6: `PAROL6_URDF/` | `models/descriptions/urdf/parol6_description/parol6_*.urdf` |
| PAROL6: native MuJoCo models | `models/descriptions/mujoco/parol6/scene_*.xml` |

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

The MSG and SSG48 hardware repositories retain their old standalone gripper
folders because the shared release currently supplies arm-mounted grippers and
validation fixtures, not a replacement gripper-only distribution. Their README
pages distinguish those older exports from the current shared models.
