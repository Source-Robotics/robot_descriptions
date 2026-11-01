# Source Robotics PAR6 URDF package

This folder is a self-contained URDF/ROS package. It can be uploaded directly
to viewers that accept a folder or ZIP because every generated URDF is at the
package root and uses ordinary relative mesh paths.

See [`DH.md`](DH.md) for the authoritative PAR6 joint-axis and DH convention.
See [KINEMATIC_REVIEW.md](KINEMATIC_REVIEW.md) for dimensioned orthographic
snapshots and tool-specific DH tables for all six configurations.

Version **2.8.1** adds a standalone authoring workflow; the physical data
introduced in 2.8.0 remains unchanged. Version **2.8.0** synchronizes the five arm link mass properties with RCB
and applies the measured 55.5 g flange, 380 g MSG and 440 g SSG48 tool masses.
See [DYNAMICS.md](DYNAMICS.md) for validation and the remaining estimated
gripper COM/inertia, motor rotor and fixed-base properties. The appearance-only
descriptions below concern visual changes; the inertial update is separate.

## Pneumatic gripper

Open `scene_pneumatic.xml` in MuJoCo or load `par6_pneumatic.urdf` in a URDF viewer. The original side-gripping assembly has two moving jaws, convex contacts and an operational TCP. Its 195 g total and finite actuation are unverified estimates; see [PNEUMATIC.md](PNEUMATIC.md).

## Actuator colors

J1, J2, and J3 have separate black motor/gearbox visuals, with the surrounding
structure kept in the original color. Edit the inline `j1_actuator_black`,
`j2_actuator_black`, or `j3_actuator_black` material's `rgba` value in each
`par6_*.urdf` to recolor an assembly independently.

The visual-only partitions are in `meshes/common/visual/`. The complete
`shoulder.STL` and `upper_arm.STL` remain the collision meshes. Link/joint
structure, transforms, inertial data, transmissions, and collision definitions
are unchanged in all three tool variants.

In the MSG variant, the camera lens and gripper motor also have independent
black visuals in `meshes/msg/visual/`. Their inline materials in
`par6_msg.urdf` are `msg_camera_lens_black` and `msg_gripper_motor_black`.
The full `meshes/msg/gripper.STL` remains the collision mesh. The camera frame,
gripper mechanism, mass, and inertia are unchanged.

## Side badge

The supplied Source Robotics / PAR6 / Collaborative artwork is fitted to the
outer upper-arm side cover in all three variants. A 1024x384 RGB texture is
painted onto the existing cover visual, with no additional geoms or triangles.
This replaces the previous seven-mesh badge. Collision assets, inertial data,
joint/body structure, actuators, and camera definitions are unchanged.

The existing `upper_arm_structure_visual` uses
`meshes/common/branding/upper_arm_structure_badged.dae`. Its embedded material
references the adjacent `par6_badge.png`; the viewer must support textured DAE
visuals. Original complete STLs continue to define collisions.

The original SVG is included alongside these assets. In the source description
repository, edit the seven `colors` entries (background, emblem, symbol,
wordmark, robot_name, tagline, divider) in
`source/artwork/par6_side_badge.json` and run `python tools/build_side_badge.py`
to rebuild both formats and every variant. Texture dimensions are configurable
in the same file. Badge colors are baked into the PNG rather than separate
runtime materials. Keep the image alongside its mesh when moving the package.

## Jaw options

| Gripper | Default | Other option |
| --- | --- | --- |
| MSG | Solid jaws | Soft TPU jaws |
| SSG48 | Solid jaws | Fin-ray jaws |

Soft TPU jaws have yellow honeycomb inserts and black holders/carriages.
SSG48 Solid jaws use the supplied `solid_jaws_pinch.STEP`; the checked surface
deviation from the matching export is below 0.055 mm. Their tip is 31.0 mm
from the mounting plane, versus 54.4 mm for Fin-ray jaws: reach is 23.4 mm shorter.

All options keep the existing finger joints, travel, coupling, drive settings,
camera and TCP. Measured tool masses are applied in version 2.8.0; tool COM and inertia remain
existing estimates. Equal mass
does not establish equal COM or inertia when a jaw's shape changes.

In `par6_motion`, the panel's **Tool and jaws** selector loads any of the five
setups, including Flange only. **Load setup** stops motion, saves the current
recording, and starts a new simulation session at the measured arm joint pose.
Controls, taught points, paths and jaw opening reset with the new session.
The configured Haply/UDP input reconnects; following must be enabled again.
Command-line examples:

```bash
python tools/jog_cartesian.py --haply --msg-jaws solid
python tools/jog_cartesian.py --haply --msg-jaws soft-tpu
python tools/jog_cartesian.py --variant ssg48 --haply --ssg48-jaws solid
python tools/jog_cartesian.py --variant ssg48 --haply --ssg48-jaws fin-ray
```

Old flag values (`stock`, `tpu-glue`, `pinch`) and model filenames remain valid
for compatibility. Python accepts the new names too, for example
`MujocoTwin('msg', msg_jaws='soft-tpu')`.

## Finger contacts and grasp checks

Each finger now uses six hidden convex collision pieces fitted through the
selected external carriage/jaw mesh. SSG48 internal transmission racks remain
visual-only; both formats share the native external contact envelope. The pieces preserve the original
surface envelope and reduce the empty space filled by a single convex hull.
The contact refinement itself preserves visible meshes and inertial definitions.
The separate version 2.8.0 update synchronizes inertia and measured masses. There are no
new bodies or joints. Contacts intentionally differ from the old single hull.
Small honeycomb holes, TPU deformation and Fin-ray flex are not simulated.

The native package includes `bench/`: four profiles, each with a 50 g box and
cylinder. From the description repository root, run:

```bash
python tools/grasp_check.py --all
python tools/grasp_check.py --profile ssg48-solid --shape box --view
```

The check closes, lifts 50 mm, holds and releases. All eight cases per robot
meet the 35 mm retained-lift and 25 mm release-drop thresholds. Objects keep
50 g mass and are shortened where needed to leave 5 mm housing clearance;
SSG48 Solid jaws use a 41.2 mm object rather than the former overlapping
60 mm fixture. Its box retains about 37.3 mm of lift. Force, friction and
pass thresholds are unchanged. See [SSG48 contact diagnosis](SSG48_CONTACTS.md)
for internal rack filtering and the fixture correction. These checks do not
establish measured hardware grip force or compliance. The command returns a
nonzero exit status when any case misses its threshold.

The source description repository includes reproducible jaw/contact builders
and the preserved STEP inputs. Stable filenames below retain older internal
identifiers; display names are the requested jaw names.

See `CONTACT_PERFORMANCE.md` for the measured collision-query cost.
See `DYNAMICS_AUDIT.md` for the measured-reference comparison and
`GRASP_FRAMES.md` for the proposed separate gripping frame.

## Choose an end effector

| End effector | Direct entry file |
| --- | --- |
| Flange only | `par6_flange.urdf` |
| MSG - Solid jaws | `par6_msg.urdf` |
| MSG - Soft TPU jaws | `par6_msg_tpu_glue.urdf` |
| SSG48 - Solid jaws | `par6_ssg48.urdf` |
| SSG48 - Fin-ray jaws | `par6_ssg48_stock.urdf` |

In every variant, `wrist_3_link` is aligned with mathematical DH frame 6,
`flange` is the physical mounting face, and `tcp` is the operational control
frame. The flange-only TCP coincides with `flange`; gripper variants retain their existing operational TCP; jaw-specific grasp
frames have not yet been calibrated. The MSG variant also includes `msg_camera_link` at the mount
pose and the REP-103-style `msg_camera_optical_frame`. The portable URDF
describes those frames but intentionally does not prescribe a simulator-specific
camera plugin.

The canonical arm joint names are `base_joint`, `shoulder_joint`,
`elbow_joint`, and `wrist_1_joint` through `wrist_3_joint`. See
[`MIGRATION.md`](MIGRATION.md) before updating code written for package 1.x.

For ROS 1, copy this directory into `catkin_ws/src`, build the workspace, source
it, and select a variant with:

```bash
roslaunch par6_description display.launch end_effector:=msg
roslaunch par6_description display.launch end_effector:=msg_tpu_glue
roslaunch par6_description display.launch end_effector:=flange
roslaunch par6_description display.launch end_effector:=ssg48
roslaunch par6_description display.launch end_effector:=ssg48_stock
```

## Important physical-data status

The original CAD exports contained zero placeholder position, effort, and
velocity limits. To avoid falsely presenting zero-width joints as fixed, the six
arm axes in these portable viewer files are represented as `continuous` joints.
This makes the model inspectable, but it is **not verified hardware-limit data**.
Use manufacturer-verified position, velocity, effort, damping, and transmission
values before commanding a physical PAR6 or relying on dynamics results.

The gripper travel ranges come from the corresponding working MJCF models.
