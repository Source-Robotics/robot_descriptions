# Changelog - Source Robotics robot descriptions

## 2.18.1 - 2026-11-03

- Organized reports by appearance, dynamics, grippers, kinematics, package checks and historical milestones; moved generated JSON out of the tools folder.
- Grouped maintenance notes, release notes and the review PDF under docs; added task-oriented indexes for documentation, tools, inputs and reports.
- Added a current model-status page and a path-migration list; refreshed links, generators and archive documentation.
- Preserved model/scene entry paths, authoring CLI paths, ROS launch commands and physical model definitions. Joint-limit and accuracy work remain separate.

## 2.18.0 - 2026-11-03

- Added dedicated PAR6/PAROL6 MoveIt 2 packages for all twelve robot/tool combinations, using the existing URDF assets.
- Added TCP IK/pose planning, OMPL collision-checked paths, named arm/gripper poses and mock ros2_control execution with coupled fingers.
- Added in-memory planning joint bounds from the captured RCB ranges for PAR6 and maintained URDF ranges for PAROL6; saved physical models stay unchanged.
- Added two downloadable configuration ZIPs and a complete installation/RViz guide; updated product entry pages.
- Added Humble/Jazzy integration jobs that plan, validate and execute trajectories from a relocated install, including negative collision/limit checks and RViz screenshots.

## 2.17.0 - 2026-11-03

- Added native ROS 2 ament/colcon installation and display launchers for both robots and standalone grippers, with tool/jaw selection, joint sliders, saved RViz views and headless/external-state options.
- Resolved ROS asset URIs in memory while preserving portable URDF model files, physical data and MuJoCo packages; retained ROS 1 manifests/build conditions and launch entry points.
- Added Humble/Jazzy Linux integration jobs covering relocated installs, all sixteen variants, nonzero external joint-state TF, mimic fingers, RViz and GUI startup, screenshots and invalid selections. Releases require those checks.
- Added ROS 2 installation/usage documentation and model-selection/resource-preservation regression checks.
- Preserved per-part RViz colors with an optional in-memory massless fixed-visual adapter; original physical frames/data and saved model files remain intact.

## 2.16.1 - 2026-11-03

- Added rendered comparisons and matching JSON examples for blue motors, custom MSG/SSG48 jaws and a graphite/teal PAR6 badge.
- Included the illustrated guide and its local images in all eight packages; linked the examples from package/product READMEs.
- Added an optional renderer that uses temporary colored model copies, preserving the active palette, runtime definitions and badge cache.
- Kept all maintained runtime model/geometry definitions and default colors unchanged.

## 2.16.0 - 2026-11-03

- Added one shared appearance palette for both robots, standalone grippers, all jaw/tool options and both formats, with individual-part overrides and badge colors.
- Added `build.py --render` for a single-command color rebuild with refreshed review snapshots, plus an applied-palette copy in every package.
- Added non-default-theme regression checks across every model, including equality of all compiled physical/geometry arrays and unchanged URDF collision/inertial/joint definitions.
- Kept existing geometry, kinematics, dynamics, contacts, controls and camera definitions. Starting colors retain the existing look, with shared precision across formats.

## 2.15.1 - 2026-11-01

- Added simulated MSG built-in camera previews for Solid and Soft TPU jaws, using the existing orange-box grasp fixtures and unmodified camera geometry/FOV.
- Included camera instructions and previews in all six arm/MSG packages and linked them from product guides. SSG48 standalone models have no camera.
- Corrected historical PAR6 appearance notes to the actual 51.5-degree vertical FOV; camera settings and all physical model data are unchanged.

## 2.15.0 - 2026-11-01

- Licensed current model/data/assets/documentation under CC BY 4.0; Python/ROS support code remains Apache-2.0. Added attribution and explicit scope to all eight packages. Earlier releases retain their existing license grants.
- Added a dedicated color guide with editable palettes, named materials, RGBA examples and regeneration steps; linked it from all four product guides.
- Removed the PAROL6 gallery footer, retaining the six robot renders and end-effector labels only.

## 2.14.0 - 2026-11-01

- Organized all eight runtime packages: detailed notes, comparison diagrams and review sheets in `docs/`; catalog images in `previews/`.
- Kept model/scene entry paths, assets, joints, collision shapes and physical properties unchanged.
- Updated catalog image paths, ROS installation, generators and portable-package link checks.
- Made PAR6/PAROL6 guide previews prominent and grouped loose repository feature PNGs under `docs/images/`.

## 2.13.0 - 2026-11-01

- Added independent MSG and SSG48 URDF/MuJoCo packages, each with both requested jaw options.
- Reused tool-relative frames, local physical properties, collision geometry and controls from the maintained robot models.
- Added standalone FK/inertia preservation, physical close/open and relocated-package validation.
- Expanded checked releases to eight model ZIPs and added product-specific `description/` entry pages for hardware repositories.
- Kept the existing PAR6/PAROL6 robot definitions and their measured mass targets unchanged.

## 2.12.1 - 2026-10-31

- Corrected SSG48 URDF collision hulls around internal transmission racks, retaining complete visuals and all physical/joint settings.
- Corrected initial housing overlap in the short-jaw grasp fixture; all 16 grasp cases and eight full-stroke contact sweeps pass.
- Added release version checks, checksums, a source manifest and tagged GitHub release automation.
- Preserved the corrected PAR6/PAROL6 geometry, shared local inertial properties and measured tool masses.

## 2.12.0 - 2026-10-31

- Added the original pneumatic gripper to PAR6 and PAROL6 in URDF and MuJoCo, with two coupled jaws, convex contacts and a retained operational TCP.
- Kept pneumatic mass/COM/inertia and finite actuation explicitly marked as estimates; retained the five earlier profiles unchanged.
- Expanded package selectors, previews, DH sheets and the printable booklet to six configurations per robot.

## 2.11.0 - 2026-10-31

- Corrected PAROL6 forearm to 177 mm and shifted its lower-arm visual/collision origins outward by 0.65 mm.
- Corrected PAROL6 flange reach to 50.5 mm using CAD housing faces to match PAR6 mounting clearance; all tools/cameras/TCPs move inward by 6.6 mm relative to J6.
- Kept PAR6 geometry/physics and inherited local inertials and finite gains unchanged.
- Regenerated model packages, previews, all ten review sheets and the printable booklet; flange-ending DH difference is now d6.

## 2.10.0 - 2026-10-31

- Added dimensioned orthographic robot diagrams and tool-specific DH tables for all ten PAR6/PAROL6 configurations.
- Included SVG/PNG/JSON review sheets in every portable package, plus an offline comparison page and optional A4 PDF booklet.
- Added reproducible 128-pose DH/URDF/MuJoCo checks and independently verified image projection.
- Kept all model definitions, geometry, physical properties, collision and drive settings unchanged.

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
- Added the jog-panel tool/jaw selector and pose-preserving session reload.
- Refined each finger to six convex contact pieces, retaining visuals and inertia.
- Added measured RCB/URDF/MJCF dynamics comparisons and eight grasp fixtures.
- Documented a separate calibrated grasp-frame proposal while preserving current TCPs.

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

## 2026-08-22 â€” Canonical model interface (2.0.0)

- Standardized joint, link, actuator, finger, tool, TCP, and MSG camera names
  across all MuJoCo and URDF variants.
- Replaced the redundant `frame6` helper with a DH-aligned `wrist_3_link`, an
  explicit physical `flange`, and the selected tool's operational `tcp`.
- Preserved the 1.x model geometry, inertia, zero pose, TCPs, camera mount, and
  positive real-hardware motion convention.

## 2026-08-22 â€” Hardware joint convention (1.1.4)

- Corrected positive J1, J3, J4, and J6 motion to follow the supplied PAR6 DH
  table and real-robot command convention in every MJCF and URDF variant.
- Documented the authoritative symbolic DH table in both portable packages.

## 2026-08-22 â€” Frames and MSG camera (1.1.3)

- Separated the mathematical DH `frame6` from each variant's operational `tcp`.
- Placed the flange-only TCP on the measured outer mounting face.
- Kept the fixed camera exclusively on the MSG MuJoCo variant and added the
  corresponding semantic camera mount to its URDF.

## 2026-08-22 â€” Portable package layout

- Added independent Menagerie-style MuJoCo and portable URDF distributions.
- Added flange, MSG, and SSG48 entry points to both formats.
- Consolidated shared meshes inside each portable distribution.
- Added Sim2Bot package-variant manifests and import selection.
- Preserved the original exports under `source/legacy/`.

All notable changes to this model will be documented in this file.


## [2026-08-22]

- Standardized root MJCF entry points and added a Sim2Bot v1 manifest.
- Added explicit MSG and SSG48 tooling variants, TCP/camera metadata, and home
  keyframes.
- Fixed legacy MJCF mesh references and removed invisible geometry that distorted
  model bounds.
- Added the declared Apache-2.0 licence file.
- Corrected Catkin package names, CMake projects, URDF launch paths, dependency
  declarations, and controlled-joint lists.
- Documented the still-unverified hardware joint, velocity, and effort limits.

## [2025-02-10]
- Added the initial files:
    - MJCF for PAR6 with SSG48 and MSG grippers
    - URDF files for 3 versions: PAR6 with flange, SSG48 gripper and MSG gripper
    - Added extra jaw options for SSG48 gripper
