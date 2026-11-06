# RAFT models: getting started

[All models](../README.md) | [PAR6](par6.md) | [Change colors](../docs/COLORS.md) | [License and attribution](../ATTRIBUTION.md)

RAFT is the teleoperation leader arm for PAR6: PAR6's kinematic structure at 5/9 scale, with a trigger on its handle
that drives the follower's gripper. It is available in **MuJoCo and URDF**, in one configuration: the arm with its
handle and trigger.

To customize appearance, edit the [shared color file](../source/appearance/colors.json) (`raft_structure`,
`raft_trigger`, `raft_horns`, `raft_fasteners`; the servos follow `motors`, or the `raft_motors` override) and
follow the [illustrated color guide](../docs/COLORS.md). One palette covers both formats and all tools.

![RAFT, the handle held in front](../docs/images/raft.png)

## 1. Download

- **MuJoCo:** [download raft_mujoco.zip](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/raft_mujoco.zip), then extract the `raft/` folder.
- **URDF:** [download raft_urdf.zip](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/raft_urdf.zip), then extract the `raft_description/` folder.

Keep each extracted folder intact; the model files use relative paths to their meshes. No CAD tools or source build
are needed to use these downloads.

| Format | Entry file |
| --- | --- |
| MuJoCo | [`scene.xml`](../mujoco/raft/scene.xml) to look at it, [`raft.xml`](../mujoco/raft/raft.xml) to put it into another MuJoCo world |
| URDF | [`raft.urdf`](../urdf/raft_description/raft.urdf) |

## 2. Open the model

**MuJoCo:** with the Python MuJoCo package installed, run this from the directory where you extracted the ZIP:

```bash
python -m mujoco.viewer --mjcf=raft/scene.xml
```

For a checkout of the central `robot_descriptions` repository, run this from its root:

```bash
python -m mujoco.viewer --mjcf=mujoco/raft/scene.xml
```

The hardware repository's `raft_description/` folder contains this guide and previews. Extract the model download
into a separate working directory or ROS workspace to use the actual model package.

**URDF viewer:** open `raft_description/raft.urdf`, keeping the `meshes/` directory alongside it.

**ROS 2 / RViz:** install the extracted `raft_description` package with colcon using
[the ROS 2 guide](../docs/ROS2.md), source your workspace, then run:

```bash
ros2 launch raft_description display.launch.py
```

This opens RViz and joint sliders. ROS 1 retains `roslaunch raft_description display.launch`.

## 3. Joints, actuators and the trigger

The joints are named like PAR6's: `base_joint`, `shoulder_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`,
`wrist_3_joint`, plus `trigger_joint`. `q = 0` is the standby pose, PAR6's standby pose, so RAFT's joint angles map
one to one onto PAR6's. `tcp` is the tool point on the handle.

The MuJoCo actuators (`<joint>_actuator`) take joint torque in Nm, as RAFT's servos run in current mode. The tendon
`j2_spring` models the optional J2 counterbalance; its stiffness 0 means no spring is fitted.

## 4. Check dimensions and frames

- [RAFT DH convention](../mujoco/raft/docs/DH.md)
- [RAFT and PAR6: the 5/9 scale](../mujoco/raft/docs/KINEMATICS_COMPARISON.md)
- [Joint ranges and where they come from](../mujoco/raft/docs/JOINT_LIMITS.md)
- [Masses, centres of mass and inertias](../mujoco/raft/docs/DYNAMICS.md)
- [Actuation and the J2 counterbalance](../mujoco/raft/docs/ACTUATION.md)

One end of every joint range is a mechanical stop measured on the first arm; the masses come from the CAD; the
servos' friction, damping and rotor inertia are estimates.

## The arm itself

RAFT's parts, BOM, assembly manual and ESP32 firmware are in the
[RAFT repository](https://github.com/Source-Robotics/RAFT-teleoperation-arm). Its firmware computes gravity
compensation from the same model reference these packages are built from
([`source/raft/`](../source/raft/README.md)).
