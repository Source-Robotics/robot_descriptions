# PAROL6 description changelog

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
