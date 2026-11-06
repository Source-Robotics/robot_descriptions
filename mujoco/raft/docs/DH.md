# RAFT DH convention

[Documentation index](README.md)

RAFT uses PAR6's kinematic structure: the standard DH (CM6) table, the same joint directions and the same zero
pose, with every length scaled to about 5/9 ([comparison](KINEMATICS_COMPARISON.md)).

- **Frames.** Link *i*'s frame is DH frame *i - 1*, and joint *i* turns about its z axis. `wrist_3_link` is DH frame
  5, and the `tcp` frame is DH frame 6.
- **Zero pose.** `q = 0` is the standby pose: upper arm vertical, forearm level and forward, wrist straight. It is
  PAR6's standby pose, and the `home` keyframe.
- **PAR6 / RCB angles.** The RCB runtime works in DH angles: theta = q + [0, -90, 180, 0, 0, 180] deg for joints 1
  to 6. They are continuous and never wrapped, as in the runtime.
- **Units.** Metres and radians in the model files; millimetres and degrees in these tables.

## DH table (model version 2.19.0)

| Joint | d (mm) | a (mm) | alpha | theta offset |
|---|---:|---:|---:|---:|
| `base_joint` | 51.220 | 13.000 | -90 deg | 0 deg |
| `shoulder_joint` | 0.000 | 100.000 | 180 deg | -90 deg |
| `elbow_joint` | 0.000 | -24.160 | 90 deg | 180 deg |
| `wrist_1_joint` | -97.920 | 0.000 | -90 deg | 0 deg |
| `wrist_2_joint` | 0.000 | 0.000 | 90 deg | 0 deg |
| `wrist_3_joint` | -78.277 | -0.000 | 180 deg | 180 deg |

The trigger (`trigger_joint`) is not part of the DH chain. Its hinge frame is fixed to `wrist_3_link` (from the CAD
assembly), and it turns about the hinge frame's z axis.

## Tool point

`tcp` sits on the trigger loop, 78.277 mm from the J6 axis along the tool axis. That is exactly 5/9 of the
140.9 mm of PAR6's SSG48 and MSG tool points in the
[robot_descriptions kinematic review](https://github.com/Source-Robotics/robot_descriptions/blob/main/docs/kinematics/robot_kinematics_review.pdf).
RAFT has no separate flange: the handle is fixed to the J6 horn.
