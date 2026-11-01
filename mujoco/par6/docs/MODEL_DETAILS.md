# Source Robotics PAR6 for MuJoCo

This is the self-contained, MuJoCo-Menagerie-style PAR6 package. Keep the whole
folder together when opening it in MuJoCo, Sim2Bot, or an online MJCF viewer.

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
structure kept in the original color. Edit the `rgba` values of
`j1_actuator_black`, `j2_actuator_black`, and `j3_actuator_black` in the selected
`par6_*.xml` to recolor each assembly independently.

The additional meshes in `assets/visual/` are visual-only: their geoms have
`contype="0"`, `conaffinity="0"`, and `mass="0"`. The original complete shoulder
and upper-arm meshes remain the collision geoms, now transparent and in group 3.
Body/joint structure, transforms, explicit inertia, contacts, actuator settings,
and keyframes are unchanged. Use these MJCF entries for MuJoCo; when importing
the separate URDF package with visuals enabled, display its visual group rather
than overlaying the full collision meshes.

The MSG variant also has independent `msg_camera_lens_black` and
`msg_gripper_motor_black` materials. Its three visual-only gripper partitions
are in `assets/visual/`; the original complete `gripper_MSG.STL` is retained
as the transparent collision geom. Camera parameters, gripper joints,
inertial data, and collision behavior are unchanged.

## Side badge

The supplied Source Robotics / PAR6 / Collaborative artwork is fitted to the
outer upper-arm side cover in all three variants. A 1024x384 RGB texture is
painted onto the existing cover visual, with no additional geoms or triangles.
This replaces the previous seven-mesh badge. Collision assets, inertial data,
joint/body structure, actuators, and camera definitions are unchanged.

The existing `upper_arm_structure_visual` uses
`assets/branding/upper_arm_structure_badged.obj` and `par6_badge.png`, through
the `par6_badge_material` material. Original displayed cover normals are
preserved for matching edge shading.

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

## Entry points

| End effector | Entry file |
| --- | --- |
| MSG - Solid jaws (default) | `scene.xml` or `scene_msg.xml` |
| Flange only | `scene_flange.xml` |
| MSG - Soft TPU jaws | `scene_msg_tpu_glue.xml` |
| SSG48 - Solid jaws | `scene_ssg48.xml` |
| SSG48 - Fin-ray jaws | `scene_ssg48_stock.xml` |

The robot definitions share one `assets/` directory. Sim2Bot reads
`sim2bot.robot.json` and can present the same entries as package variants.

All robot geometry is nested below the movable `base_link` body. In every
variant, `wrist_3_link` is aligned with mathematical DH frame 6, `flange` is the
physical mounting face, and `tcp` is the operational control frame. The
flange-only TCP coincides with `flange`; MSG and SSG48 retain their existing operational TCP for IK, telemetry,
and reference-frame visualization; jaw-specific grasp frames are not yet calibrated.

Only the MSG variant contains the fixed `msg_camera`. Its camera and
semantic frames do not use visible helper geometry. The current camera mount
pose and 90-degree field of view should be checked against production CAD and
camera calibration before using rendered images as calibrated measurements.

## Quick test

Open one of the `scene_*.xml` files. Do not open a robot XML without also
uploading the `assets/` directory, because the model uses relative mesh paths.

The authored arm ranges in these MJCF files are permissive simulation ranges;
they are not a replacement for hardware safety limits.
