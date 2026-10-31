# Changelog

## 2.12.1 - 2026-10-31

- Corrected SSG48 URDF contact proxies to omit internal rack collisions while preserving complete visuals, mass properties and joint/drive settings.
- Corrected short-jaw grasp fixture clearance; all eight grasp cases per robot pass with unchanged force, friction and pass thresholds.
- Added 201-point internal collision sweeps and grasp regression checks to the standalone build.

## 2.12.0 - 2026-10-31

- Added the original pneumatic gripper to PAR6 and PAROL6 in URDF and MuJoCo, with two coupled jaws, convex contacts and a retained operational TCP.
- Kept pneumatic mass/COM/inertia and finite actuation explicitly marked as estimates; retained the five earlier profiles unchanged.
- Expanded package selectors, previews, DH sheets and the printable booklet to six configurations per robot.

## 2.11.0 - 2026-10-31

- Corrected PAROL6 forearm to 177 mm and wrist flange reach to 50.5 mm, matching PAR6 housing-to-tool spacing.
- Regenerated comparison/review documents; PAR6 model geometry, physics and drive settings are unchanged.

## 2.10.0 - 2026-10-31

- Added dimensioned orthographic model snapshots and tool-specific standard-DH tables for every PAR6/PAROL6 configuration.
- Included SVG, PNG, machine-readable transforms and 128-pose numerical cross-checks in every package.
- Added an offline two-model comparison page and optional ten-page printable booklet.
- Documented retained operational TCPs across jaw variants; no model geometry, kinematics, dynamics or collision settings changed.

## 2.9.1 - 2026-10-31

- Standardized public PAROL6 joint axes and origin orientations to PAR6, including J6.
- Retained the 176.35 mm PAROL6 forearm; only standard-DH d4 differs.
- Converted historical export joint limits and documented saved-trajectory migration.
- Preserved physical properties, meshes, tool subtrees and finite gains; added actuation documentation.
- PAR6 model physics and kinematics are unchanged.

## 2.9.0 - 2026-10-31

- Added standalone PAROL6 URDF/MuJoCo packages with all five shared tool profiles.
- Preserved the supplied PAROL6 arm geometry/kinematics; inherited PAR6 physical properties and black actuator appearance.
- Added numeric PAR6/PAROL6 DH tables, joint-direction and wrist-frame diagrams, and multi-pose validation.
- PAR6 model kinematics, physical parameters and existing tool profiles are unchanged.

## 2.8.1 - 2026-10-31

- Made model generation, dynamics auditing, validation, grasp checks and
  benchmarking independent of the IK/RCB/application directory arrangement.
- Added a versioned RCB reference snapshot, regression fixtures, an explicit
  upstream import command and a NumPy reference evaluator checked against RCB.
- Added a full build entry point, pinned dependency lock, standalone tests,
  clean-checkout documentation and a GitHub Actions build workflow.
- Retained the measured masses, existing physics/calibration status and
  self-contained URDF/MuJoCo release packages.

## 2.8.0 - 2026-10-28

- Synchronized all five arm link masses, COM and full central inertia tensors
  with the preferred RCB reference, using each format's existing body frames.
- Applied measured tool totals: flange 55.5 g, MSG 380 g, SSG48 440 g, with
  39.3 g per moving finger/jaw/slider assembly on both grippers.
- Preserved tool COM/tensors as unverified estimates, motor armature, fixed
  base, geometry, collisions, joint frames/limits and drive parameters.
- Added repeatable read-only reference export, jaw-rebuild integration,
  160-pose checks per profile, measured-mass fixtures and updated distributions.

## 2.7.0 - 2026-10-27

- Named MSG options Solid jaws / Soft TPU jaws and SSG48 options Solid jaws / Fin-ray jaws.
- Added a jog-panel tool/jaw selector with pose-preserving session reload.
- Refined finger collisions to six convex pieces each, retaining visible meshes and inertia.
- Added eight close/lift/hold/release fixtures and measured-reference dynamics audit.
- Preserved TCPs and documented a separate jaw-specific grasp-frame proposal.

## 2.6.0 - 2026-10-27

- Made solid pinch fingers the default SSG48 geometry in MJCF and URDF, matched
  against the supplied STEP and combined with each original finger carriage.
- Preserved the previous SSG48 fingers as a stock option and added
  `--ssg48-jaws pinch|stock` to the Cartesian jog app and portable variants.
- Kept joint travel, coupling, TCP, actuator settings, and existing inertial
  values; updated collision geometry to follow the selected fingers.
- Added STEP/mesh verification, pose and closing-simulation checks, previews,
  and updated portable packages. No extra bodies, joints, or geoms are needed.

## 2.5.0 - 2026-10-27

- Added optional MSG TPU honeycomb jaws from the supplied two-solid STEP,
  with yellow inserts, black holders, and the original finger carriages.
- Kept the stock MSG entries and added selectable native MJCF/URDF entries,
  Sim2Bot variants, and `--msg-jaws stock|tpu-glue` in the Cartesian jog app.
- Generated collision hulls from the selected geometry while preserving the
  current finger inertial data, joint frames/travel/coupling, camera, TCP, and
  actuator settings. Contact geometry intentionally differs from stock.
- Added reproducible STEP conversion, configurable colors/placement, a jaw
  preview, physical validation, and local performance measurements.

## 2.4.1 - 2026-10-27

- Replaced the seven-mesh side badge with a configurable 1024x384 PNG on the
  existing upper-arm cover visual, removing seven geoms and 9,916 triangles.
- Preserved the original cover geometry and displayed edge shading, with
  native textured OBJ/MJCF and DAE/URDF assets and the original SVG source.
- Kept seven editable palette entries; color changes require rebuilding the
  PNG through the shared badge builder.
- Revalidated unchanged physics and measured 5.30 ms/frame versus 5.60 ms for
  the previous badge and 5.31 ms without a badge in the local offscreen test.

## 2.4.0 - 2026-10-27

- Added the supplied Source Robotics / PAR6 / Collaborative SVG to the outer
  upper-arm side cover in all three URDF and MJCF variants.
- Kept the background, emblem, symbol, wordmark, robot name, tagline, and divider
  independently recolorable through seven named materials and a shared palette.
- Preserved the source SVG and added a reproducible vector-mesh builder with
  checks that the badge stays within the flat panel face.
- Confirmed unchanged collision, inertia, kinematics, dynamics, camera, and
  actuator definitions; refreshed portable distributions and previews.

## 2.3.1 - 2026-10-27

- Added independently colored black camera lens and motor visuals to the MSG
  gripper in URDF and MJCF, preserving original triangle records and collision,
  inertia, kinematics, camera, and actuator definitions.
- Extended physical-equivalence validation and local performance measurements.
- Updated the MSG preview and portable distributions.

## 2.3.0 - 2026-10-27

- Separated J1, J2, and J3 motor/gearbox visuals with independent black materials
  in the flange, MSG, and SSG48 URDF and MJCF variants.
- Preserved every original mesh triangle, collision asset, inertial definition,
  joint/body frame, actuator setting, and kinematic interface.
- Added reproducible mesh partitioning and physical-equivalence verification.
  MuJoCo 3.3.0 comparisons matched exactly across 26 poses and 300 simulation
  steps per variant, including collision contacts.
- Refreshed package previews and portable ZIP distributions.

## 2.1.0 â€” 2026-08-24

- Added a `parked` keyframe to all three MJCF variants: a folded rest pose that
  tucks the arm using J2 (`-0.5` rad) and J3 (`-1.3` rad), leaving J1 and the
  wrist at zero. Measured envelope drops from 0.341 m at `home` to 0.189 m
  (MSG), with no self-collision, and the pose holds under gravity through the
  position actuators (< 0.005 rad drift over 3000 steps).

## 2.0.0 â€” 2026-08-22

- Standardized semantic arm, wrist, tool, finger, actuator, and camera names.
- Made `wrist_3_link` the authoritative DH frame-6 link and added an explicit
  `flange` mounting frame without changing the CAD, TCP, camera, or inertial
  poses.
- Added a documented 1.x-to-2.0 migration map shared with the URDF package.

## 1.1.4 â€” 2026-08-22

- Corrected J1, J3, J4, and J6 so positive commands follow the authoritative
  Source Robotics DH and real-hardware rotation convention.
- Standardized all six arm joints on their local positive Z axes in MJCF and
  URDF without changing link origins or zero-configuration geometry.
- Added the authoritative symbolic DH table and frame-semantics documentation.

## 1.1.3 â€” 2026-08-22

- Kept `frame6` at the mathematical DH origin and moved the flange-only `tcp`
  to the measured outer mounting face.
- Standardized the MSG fixed camera and removed the accidental camera copy from
  the SSG48 variant.
- Added matching DH, TCP, and MSG camera semantic frames to the URDF package.

## 1.1.2 â€” 2026-08-22

- Aligned the flange, MSG, and SSG48 TCP orientation with PAR6 DH frame 6.
- Added the same semantic frame to the flange-only URDF variant.

## 1.1.1 â€” 2026-08-22

- Attached the base mesh and arm chain to one movable `base_link` root body.
- Corrected MSG and SSG48 TCP positions to match their URDF grasp frames.
- Removed invisible camera test-box geometry from runtime models.

## 1.1.0 â€” 2026-08-22

- Added explicit flange, MSG, and SSG48 scene entry points.
- Consolidated all variants onto one shared asset directory.
- Added Sim2Bot package-variant metadata.
- Added a real flange-only MJCF definition.

## 1.0.0 â€” 2025-02-10

- Initial PAR6 MJCF export.
