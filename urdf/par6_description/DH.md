# PAR6 DH convention

The PAR6 joint convention in this package follows the Source Robotics standard
DH definition below. `TH1` through `TH6` are the real-robot joint coordinates;
positive simulated joint motion has the same sign as positive hardware motion.

| Joint | Link | theta | alpha | r / a | d |
| ---: | --- | --- | --- | --- | --- |
| 1 | Base | `TH1` | `-pi/2` | `a2` | `a1` |
| 2 | Shoulder | `TH2 - pi/2` | `pi` | `a3` | `0` |
| 3 | Elbow | `TH3 + pi` | `pi/2` | `-a4` | `0` |
| 4 | Wrist 1 | `TH4` | `-pi/2` | `0` | `-a5` |
| 5 | Wrist 2 | `TH5` | `pi/2` | `0` | `0` |
| 6 | Wrist 3 | `TH6 + pi` | `pi` | `-a7` | `-a6` |

Positive simulated motion follows `TH1` through `TH6` exactly. Joints 1–5 use
local axis `0 0 1`. Joint 6 uses `0 0 -1` in MJCF/URDF because its child,
`wrist_3_link`, is aligned directly with DH frame 6 and the final DH row includes
`alpha = pi`. This representation preserves the positive hardware direction;
the negative local axis is a coordinate-expression detail, not a sign reversal.

The existing CAD-derived link origins remain authoritative for geometry.
`wrist_3_link` is the DH frame-6 link. Its fixed child `flange` is the physical
tool-mounting face, and `tcp` is the selected tool's operational frame. A tool
offset must not be folded into the six hardware joint coordinates.

See [the PAR6/PAROL6 comparison](KINEMATICS_COMPARISON.md) for numeric DH
tables, the supplied PAROL6 joint-direction map, its wrist-frame adapter and
the 0.65 mm forearm dimension difference. This comparison does not change
PAR6's existing hardware convention.
