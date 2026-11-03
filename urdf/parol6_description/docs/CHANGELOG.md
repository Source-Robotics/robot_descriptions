# PAROL6 description changelog

## 2.18.1 - 2026-11-03

- Added native ROS 2 installation, RViz display and joint sliders for all six tool profiles.
- Retained ROS 1 compatibility and portable model/physical definitions.

## 2.16.1 - 2026-11-03

- Added illustrated color examples for motors, jaws and badge, bundled with the package.
- Kept runtime models, default colors and physical properties unchanged.

## 2.16.0 - 2026-11-03

- Added a shared color palette and individual-part overrides across both formats and all tools.
- Kept all physical model and camera definitions unchanged.

## 2.15.1 - 2026-11-01

- Added native MSG camera previews and rendering instructions to the package documentation.
- Kept model camera settings, geometry, physical properties and controls unchanged.

## 2.15.0 - 2026-11-01

- Licensed model data/assets under CC BY 4.0, retaining Apache-2.0 for support code.
- Added packaged attribution and color-customization documentation links.

## 2.14.0 - 2026-11-01

- Moved detailed notes and review sheets into docs/, and catalog images into previews/.
- Retained runtime model, scene, mesh and ROS launch entry paths and all physical data.

## 2.13.0 - 2026-10-31

- Corrected SSG48 URDF collision proxies to omit internal transmission racks while preserving their visuals and all inertial properties.
- Corrected short-jaw grasp fixture clearance and added full-stroke internal contact and close/lift/hold/release regression checks.

## 2.12.0 - 2026-10-31

- Added the original pneumatic gripper with independently moving, coupled jaws, collision pieces, finite actuation and estimated CAD mass distribution.
- Included the sixth profile in packages, previews, DH tables and review sheets.

## 2.11.0 - 2026-10-31

- Corrected a5 to 177 mm, moving the wrist origin and lower-arm visual/collision placement outward by 0.65 mm.
- Matched PAR6 housing-to-tool spacing: PAROL6 flange reach is 50.5 mm, moving every tool and TCP inward by 6.6 mm relative to J6.
- Kept public axes, origin orientations, flange-relative tool subtrees, local inertials and finite actuator settings.
- Regenerated all review diagrams, DH tables and model packages.

## 2.10.0 - 2026-10-31

- Added dimensioned orthographic snapshots and tool-specific DH review tables.

## 2.9.1 - 2026-10-31

- Standardized axes/origin orientations to PAR6; retained the historical 176.35 mm forearm and initially shared the 57.1 mm flange reach, superseded above.

## 2.9.0 - 2026-10-31

- First PAROL6 package with shared tools and physical properties, using the supplied export angle convention.
