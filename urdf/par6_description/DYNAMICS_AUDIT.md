# RCB / MuJoCo / URDF mass-property comparison

RCB arm link mass, COM and central inertia are the user-preferred reference. This report changes no dynamics data.
Motor rotor inertias in RCB and simulation are uncalibrated estimates; the user has deferred correcting them.
COM and inertia are transformed into matching DH frames before comparison.

Reference: the versioned `source/dynamics/rcb_reference.json` snapshot captured 2026-10-31. Builds do not import a local runtime or motion application.
Upstream arm data originates in `robots/PAR6.py`, tool data in `grippers/*.xml`, and limits in `robots/PAR6.xml`. Source hashes are recorded in the snapshot. Refresh only with `tools/import_rcb_reference.py --rcb-root PATH --date YYYY-MM-DD`.
The live dynamics builder is `utility/pinocchio_attach.py::build_pinocchio_dynamics`; it consumes those RTB link mass/COM/inertia values and reflected rotor inertia. A separate kinematic helper uses placeholder inertias and is not the measured dynamics reference.

## Physical mass updates reported 2026-10-28

Physical measurements are recorded in `source/dynamics/physical_measurements.json`. The tables compare the currently configured RCB and exported model values.
Application status: Applied to all five URDF/MJCF variants on 2026-10-28. Working RCB runtime tool configuration remains unchanged. See dynamics_sync_validation.json. Your RCB runtime files are read-only for this export.

| Tool | Physical total, g | Each moving assembly, g | Fixed assembly, g |
|---|---:|---:|---:|
| flange | 55.5 | pending | n/a or pending |
| msg | 380.0 | 39.3 | 301.4 |
| ssg48 | 440.0 | 39.3 | 361.4 |

MSG moving assemblies include finger, jaw and sliding elements; the 39.3 g figure is approximate.

## flange (Flange)

| Link | RCB mass g | MJCF mass g | URDF mass g | MJCF COM delta mm | URDF COM delta mm | MJCF inertia relative delta | URDF inertia relative delta |
|---|---:|---:|---:|---:|---:|---:|---:|
| J1 | 1993.140 | 1993.140 | 1993.140 | 0.000 | 0.000 | 0.0% | 0.0% |
| J2 | 1332.240 | 1332.240 | 1332.240 | 0.000 | 0.000 | 0.0% | 0.0% |
| J3 | 685.110 | 685.110 | 685.110 | 0.000 | 0.000 | 0.0% | 0.0% |
| J4 | 579.560 | 579.560 | 579.560 | 0.000 | 0.000 | 0.0% | 0.0% |
| J5 | 523.810 | 523.810 | 523.810 | 0.000 | 0.000 | 0.0% | 0.0% |
| complete_tool | 55.500 | 55.500 | 55.500 | 8.046 | 8.046 | n/a | n/a |

Largest gravity torque differences over 25 poses, J1-J6 (Nm): `[0.0, 0.004, 0.004, 0.001, 0.004, 0.0]`.
Largest relative arm mass-matrix difference: 5.3%.
This includes the existing estimated rotor/armature settings; see JSON for both arrays.
With rotor terms removed from both models, the largest relative arm mass-matrix difference is 0.4%.

TCP maximum position/angle error against RCB: `{'mujoco': {'position_m': 2.5545220552182024e-06, 'angle_rad': 8.904398594191226e-06}, 'urdf': {'position_m': 7.21061038483164e-07, 'angle_rad': 1.5063370459637487e-14}}` (metres/radians).
Joint-limit arrays are in the JSON. Portable URDF arm joints remain continuous and native MJCF ranges are permissive simulation ranges; neither is a copy of the RCB hardware limits.

## msg (MSG_small_motor_150mm_rail)

| Link | RCB mass g | MJCF mass g | URDF mass g | MJCF COM delta mm | URDF COM delta mm | MJCF inertia relative delta | URDF inertia relative delta |
|---|---:|---:|---:|---:|---:|---:|---:|
| J1 | 1993.140 | 1993.140 | 1993.140 | 0.000 | 0.000 | 0.0% | 0.0% |
| J2 | 1332.240 | 1332.240 | 1332.240 | 0.000 | 0.000 | 0.0% | 0.0% |
| J3 | 685.110 | 685.110 | 685.110 | 0.000 | 0.000 | 0.0% | 0.0% |
| J4 | 579.560 | 579.560 | 579.560 | 0.000 | 0.000 | 0.0% | 0.0% |
| J5 | 523.810 | 523.810 | 523.810 | 0.000 | 0.000 | 0.0% | 0.0% |
| complete_tool | 370.000 | 380.000 | 380.000 | 5.849 | 6.208 | 28.1% | 24.3% |

Largest gravity torque differences over 25 poses, J1-J6 (Nm): `[0.0, 0.043, 0.035, 0.019, 0.022, 0.02]`.
Largest relative arm mass-matrix difference: 3.8%.
This includes the existing estimated rotor/armature settings; see JSON for both arrays.
With rotor terms removed from both models, the largest relative arm mass-matrix difference is 1.3%.

TCP maximum position/angle error against RCB: `{'mujoco': {'position_m': 3.0935994823844436e-06, 'angle_rad': 8.904398594191226e-06}, 'urdf': {'position_m': 3.1691857816043674e-06, 'angle_rad': 8.624398452343e-06}}` (metres/radians).
Joint-limit arrays are in the JSON. Portable URDF arm joints remain continuous and native MJCF ranges are permissive simulation ranges; neither is a copy of the RCB hardware limits.

## ssg48 (SSG48)

| Link | RCB mass g | MJCF mass g | URDF mass g | MJCF COM delta mm | URDF COM delta mm | MJCF inertia relative delta | URDF inertia relative delta |
|---|---:|---:|---:|---:|---:|---:|---:|
| J1 | 1993.140 | 1993.140 | 1993.140 | 0.000 | 0.000 | 0.0% | 0.0% |
| J2 | 1332.240 | 1332.240 | 1332.240 | 0.000 | 0.000 | 0.0% | 0.0% |
| J3 | 685.110 | 685.110 | 685.110 | 0.000 | 0.000 | 0.0% | 0.0% |
| J4 | 579.560 | 579.560 | 579.560 | 0.000 | 0.000 | 0.0% | 0.0% |
| J5 | 523.810 | 523.810 | 523.810 | 0.000 | 0.000 | 0.0% | 0.0% |
| complete_tool | 414.840 | 440.000 | 440.000 | 20.881 | 20.881 | 27.9% | 27.9% |

Largest gravity torque differences over 25 poses, J1-J6 (Nm): `[0.0, 0.069, 0.046, 0.019, 0.065, 0.001]`.
Largest relative arm mass-matrix difference: 6.6%.
This includes the existing estimated rotor/armature settings; see JSON for both arrays.
With rotor terms removed from both models, the largest relative arm mass-matrix difference is 5.1%.

TCP maximum position/angle error against RCB: `{'mujoco': {'position_m': 3.080888761319944e-06, 'angle_rad': 8.904398594191226e-06}, 'urdf': {'position_m': 3.4596975227391915e-06, 'angle_rad': 1.0913461085021562e-05}}` (metres/radians).
Joint-limit arrays are in the JSON. Portable URDF arm joints remain continuous and native MJCF ranges are permissive simulation ranges; neither is a copy of the RCB hardware limits.

## Interpretation

- Version 2.8 synchronizes arm link properties and measured simulation tool masses. RCB runtime tool masses remain at their older configured values.
- Whole-robot discrepancies below include unresolved tool COM/inertia and estimated rotor terms. The separate dynamics_sync_validation.json isolates and validates the five-link arm without these terms.
- Close mass values alone do not establish matching COM or inertia.
- RCB tool inertia is aggregated; simulation fingers move. The reference tool opening is unspecified.
- A dynamics synchronization should convert RCB DH inertias into each CAD body frame, account for fixed/moving tool aggregation, and validate gravity and mass matrices. Raw tensor copying is incorrect.
- Measured-reference data in RCB-Runtime was not edited.
