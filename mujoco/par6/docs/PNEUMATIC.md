# Pneumatic gripper

The original PAROL6 L6 assembly is available as `pneumatic` on both arms.
Its preserved source triangles are partitioned into the housing/air fittings
and two moving jaw assemblies. Black jaws and a silver housing are visuals;
convex contact pieces follow the actual selected geometry.

## Load and move

Open `scene_pneumatic.xml` in either MuJoCo package, for example:

```bash
python -m mujoco.viewer --mjcf scene_pneumatic.xml
```

URDF entries are `par6_pneumatic.urdf` and `parol6_pneumatic.urdf`.
The ROS display launch accepts `end_effector:=pneumatic`.

`left_finger_joint` and `right_finger_joint` are opposing prismatic joints.
Both use displacement 0 for open and +0.0035 m for closed; the right joint
mimics the left in URDF and is coupled to it by a native equality constraint.
Set MuJoCo `gripper_actuator` control to 0 or 0.0035 m for the endpoints.
The CAD inner clearance is approximately 7.1 mm open and 0.1 mm closed.

The force-limited position actuator approximates valve-command endpoints.
It does not model compressed air, pressure-dependent force, or valve dynamics.
Kp = 8000 N/m, actuator force limit = +/-20 N, and finger damping = 2 Ns/m
are simulation estimates, not verified hardware ratings. URDF controller gains
belong in the consuming simulator/controller; its joint limits are included.

## Mount and TCP

The assembly rear face is source Z = -36 mm. Mesh origins translate it
+36 mm to body Z = 0, then the body mounts 15 mm behind the common flange
reference with a 180-degree Y rotation. This keeps the same approximately
1 mm housing clearance as the other shared tools. The flange offset is
57.1 mm on PAR6 and 50.5 mm on PAROL6; no raw wrist-specific offset is baked
into the pneumatic meshes.

The operational TCP follows the legacy MHZ2-10D reference point in the source
assembly: XYZ (+45.25, 0, -62.8) mm. Rebasing/mounting carries that point with
the tool. J6-to-TCP axial reach is 68.9 mm on PAR6 and 62.3 mm on PAROL6;
the lateral offset is 45.25 mm. Its orientation matches the other package
TCPs. This is a retained operational reference, not an independently measured
grasp center. Source geometry includes a small jaw asymmetry.

## Estimated physical data

The legacy RCB MHZ2-10D configuration lists a **195 g total marked TODO**.
No pneumatic mass measurement was supplied. That total is distributed by
solid CAD volume; each rigid part's COM and full central inertia are computed
from its CAD volume with the resulting uniform density. These are explicitly
unverified simulation estimates, separate from measured MSG/SSG48 masses.

| Rigid assembly | Estimated mass (g) | Convex contact pieces |
| --- | ---: | ---: |
| pneumatic_gripper_link | 176.788 | 6 |
| left_finger_link | 9.106 | 6 |
| right_finger_link | 9.106 | 6 |

The authoring inputs are `source/pneumatic/model.json` and the byte-preserved
`source/parol6/original/meshes/L6.STL`. Normal builds need no sibling API or RCB
checkout. See `kinematics_review/pneumatic.json` for FK checks and transforms.
