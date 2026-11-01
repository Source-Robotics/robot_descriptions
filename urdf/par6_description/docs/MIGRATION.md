# PAR6 1.x to 2.0 migration

Version 2.0 standardizes the public model interface. Geometry, inertial data,
zero pose, TCP poses, camera mount, and positive joint directions are preserved,
but scripts that address elements by name must use the new names.

| 1.x name | 2.0 name |
| --- | --- |
| `shoulder_JOINT` | `base_joint` |
| `upper_arm_JOINT` | `shoulder_joint` |
| `elbow_JOINT` | `elbow_joint` |
| `lower_arm_JOINT` | `wrist_1_joint` |
| `wrist_JOINT` | `wrist_2_joint` |
| `gripper_JOINT` | `wrist_3_joint` |
| `shoulder` | `shoulder_link` |
| `upper_arm` | `upper_arm_link` |
| `elbow` | `forearm_link` |
| `lower_arm` | `wrist_1_link` |
| `wrist` | `wrist_2_link` |
| `frame6` | `wrist_3_link` |
| `gripper` (flange variant) | `flange_link` |
| `gripper` (MSG variant) | `msg_gripper_link` |
| `gripper` (SSG48 variant) | `ssg48_gripper_link` |
| `joint_jaw1` / `jaw1_JOINT` | `left_finger_joint` |
| `joint_jaw2` / `jaw2_JOINT` | `right_finger_joint` |

The frame chain is now `wrist_3_link -> flange -> selected tool -> tcp`.
`wrist_3_link` is mathematical DH frame 6; `flange` is the physical mounting
face; `tcp` is the Cartesian-control frame. The MSG model additionally provides
`msg_camera_link` and `msg_camera_optical_frame`.
