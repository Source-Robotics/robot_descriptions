# Dimensioned kinematic review

[Open the offline two-model comparison](index.html) or [the twelve-page printable booklet](../../../docs/kinematics/robot_kinematics_review.pdf).

Two robots, each with six configurations: flange, MSG Solid jaws, MSG Soft TPU jaws, SSG48 Solid jaws, SSG48 Fin-ray jaws and the original side-gripping pneumatic gripper. Each sheet shows the actual native visual meshes at zero model-joint angles, using one orthographic camera and scale. No mesh dimensions were measured from pixels. The dimensions and TCP transforms come from URDF.

| Robot / configuration | a1 | a2 | a3 | a4 | a5 | a6 | a7 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| PAR6 / Flange only | 110.5 | 23.420721 | 180 | 43.5 | 177 | 57.1 | 0 |
| PAR6 / MSG / Solid jaws | 110.5 | 23.421 | 180 | 43.5 | 177 | 140.9 | 28.35 |
| PAR6 / MSG / Soft TPU jaws | 110.5 | 23.421 | 180 | 43.5 | 177 | 140.9 | 28.35 |
| PAR6 / SSG48 / Solid jaws | 110.5 | 23.421 | 180 | 43.5 | 177 | 140.9 | 0 |
| PAR6 / SSG48 / Fin-ray jaws | 110.5 | 23.421 | 180 | 43.5 | 177 | 140.9 | 0 |
| PAR6 / Pneumatic gripper | 110.5 | 23.420721 | 180 | 43.5 | 177 | 68.9 | 45.25 |
| PAROL6 / Flange only | 110.5 | 23.420721 | 180 | 43.5 | 177 | 50.5 | 0 |
| PAROL6 / MSG / Solid jaws | 110.5 | 23.421 | 180 | 43.5 | 177 | 134.3 | 28.35 |
| PAROL6 / MSG / Soft TPU jaws | 110.5 | 23.421 | 180 | 43.5 | 177 | 134.3 | 28.35 |
| PAROL6 / SSG48 / Solid jaws | 110.5 | 23.421 | 180 | 43.5 | 177 | 134.3 | 0 |
| PAROL6 / SSG48 / Fin-ray jaws | 110.5 | 23.421 | 180 | 43.5 | 177 | 134.3 | 0 |
| PAROL6 / Pneumatic gripper | 110.5 | 23.420721 | 180 | 43.5 | 177 | 62.3 | 45.25 |

All lengths above are millimetres. Standard DH uses `Rz(theta) Tz(d) Tx(a) Rx(alpha)`. The final row includes the configured TCP. Its theta expressions already contain the required joint offsets; do not apply those offsets twice.

## Numerical checks

Each table was evaluated at 128 reproducible poses, including zero and lower/upper bounds. Revolute joints use their declared limits; continuous joints use [-pi, pi] for this check. These checks compare model consistency, not real-hardware calibration.

| Comparison | Maximum translation (mm) | Maximum rotation (deg) |
| --- | ---: | ---: |
| Actual URDF vs native MuJoCo TCP | 0.004340 | 0.001350 |
| Ideal standard DH vs actual URDF TCP | 0.005262 | 0.001048 |
| Ideal standard DH vs native MuJoCo TCP | 0.005366 | 0.001350 |

The right-angle DH table approximates the rounded rotations in some exports. Shoulder offset is 23.420721 mm in the flange URDF and 23.421 mm in several tool profiles. These precision differences are retained and their accumulated FK effect is reported above.

Snapshot marker registration was checked against three temporary OpenGL markers for every image; maximum error was 0.409 pixels. The final snapshots exclude these temporary markers.

## Items to review visually

- Both robots use a5 = 177 mm (DH d4 = -177 mm). PAROL6 lower-arm visual/collision placement and the wrist origin move outward 0.65 mm relative to the historical export.
- PAR6 flange-only a6 = 57.1 mm; PAROL6 a6 = 50.5 mm. Matching housing-to-tool spacing moves every PAROL6 tool inward 6.6 mm. The spacing correction is derived from CAD housing faces, not an independent hardware measurement.
- With either MSG or SSG48, a6 = 140.9 mm on PAR6 and 134.3 mm on PAROL6. MSG a7 = 28.35 mm; SSG48 and flange-only a7 = 0.
- Pneumatic a6 = 68.9 mm on PAR6 and 62.3 mm on PAROL6, with a7 = 45.25 mm. Its operational TCP comes from the legacy reference. Its 195 g total, CAD mass distribution and finite servo settings are unverified estimates.
- Jaw variants within each gripper share the same operational TCP. Identical DH tables do not imply identical contact geometry. SSG48 Solid jaws are about 23.4 mm shorter than Fin-ray jaws.
- These are physical joint-center dimensions, not intermediate DH frame origins. J5/J6 centers coincide. Zero pose is a wrist singularity chosen for the drawing; the numerical checks also use nonzero angles.

All four packages contain an independent copy of their six SVG/PNG sheets plus JSON transforms, source hashes and diagnostics. PAROL6 forearm geometry and the entire flange subtree use the corrected placements. Original mesh records, local mass/COM/tensors and finite drive settings are retained. PAR6 model geometry and physics are unchanged.
