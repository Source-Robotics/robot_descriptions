# Changelog

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
