# Masses, centres of mass and inertias

[Documentation index](README.md)

The mass properties are SolidWorks' (RAFT inertials.pdf, with the masses corrected to the printed parts at the
print table's settings). Each link includes its printed parts, the servo it carries, the horn it is fixed to and
the screws. They are given in each link's own frame; the inertias are about the centre of mass, and both formats
carry the full tensor, off-diagonal terms included.

| Link | Mass (g) | COM in the link frame (mm) |
|---|---:|---|
| `base_link` | 87.76 | -7.30, -0.42, 10.52 |
| `shoulder_link` | 43.14 | 6.83, -0.10, 48.72 |
| `upper_arm_link` | 18.20 | 0.00, -50.00, -23.15 |
| `forearm_link` | 71.40 | 14.84, 7.44, 2.50 |
| `wrist_1_link` | 46.10 | -0.02, 21.35, -71.39 |
| `wrist_2_link` | 35.10 | 4.64, 0.03, 0.48 |
| `wrist_3_link` | 55.30 | 1.62, 26.17, -45.71 |
| `trigger_link` | 14.30 | -13.10, 15.20, 12.78 |
| total | 371.30 | |

## Status

| Item | Status |
|---|---|
| Link masses | From CAD. Not yet weighed: printed parts vary with filament and infill. |
| Centres of mass and inertias | From CAD. |
| Base | From CAD. The ESP32-PoE board and the cables are not included. |

RAFT's firmware compensates gravity with the same numbers: one generator in the RAFT repository
([`firmware/tools/gen_model.py`](https://github.com/Source-Robotics/RAFT-teleoperation-arm/blob/main/firmware/tools/gen_model.py)) writes the firmware's model and the
reference these packages are built from, so a mass that is off shows the same way in both. Weighing each printed
link with its servo and horn is the next improvement.
