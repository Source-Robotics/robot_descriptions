# RAFT and PAR6

[Documentation index](README.md)

RAFT is a teleoperation leader for PAR6: the same kinematic structure, joint directions and zero pose, scaled to
5/9. Joint angles therefore map one to one, which is how the firmware teleoperates PAR6.

| Length | RAFT (mm) | PAR6 (mm) | RAFT / PAR6 |
|---|---:|---:|---:|
| a2 | 13.000 | 23.4207 | 0.5551 |
| a3 | 100.000 | 180 | 0.5556 |
| a4 | 24.160 | 43.5 | 0.5554 |
| a5 | 97.920 | 177 | 0.5532 |
| a6 (TCP) | 78.277 | 140.9 | 0.5556 |

PAR6's lengths are those of the
[robot_descriptions kinematic review](https://github.com/Source-Robotics/robot_descriptions/blob/main/docs/kinematics/robot_kinematics_review.pdf),
with the SSG48 or MSG tool point for a6. RAFT's base (d1 = 51.22 mm against PAR6's 110.5 mm) is lower than a pure
scale would make it; the base height does not change any joint angle.

Not scaled: the joint ranges (RAFT's mechanical stops, [joint ranges](JOINT_LIMITS.md)), the masses, and the
actuation (RAFT's servos run in current mode, [actuation](ACTUATION.md)).
