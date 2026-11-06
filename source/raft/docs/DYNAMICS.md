# Masses, centres of mass and inertias

[Documentation index](README.md)

The mass properties are SolidWorks' (RAFT inertials.pdf, with the masses corrected to the printed parts at the
print table's settings). Each link includes its printed parts, the servo it carries, the horn it is fixed to and
the screws. They are given in each link's own frame; the inertias are about the centre of mass, and both formats
carry the full tensor, off-diagonal terms included.

{{MASS_TABLE}}

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
