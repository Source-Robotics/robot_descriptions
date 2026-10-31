# Dynamics data in version 2.8.0

Updated 2026-10-28. The five moving arm link mass properties in both URDF and
MuJoCo now come from the user's preferred RCB model. The exporter converts
COM and the complete central inertia tensor from RCB DH coordinates into
each format's existing CAD body coordinates. It retains the joint frames,
TCPs, visible meshes, collision geometry, drive parameters and jaw options.

| Arm link | Mass, g |
| --- | ---: |
| J1 / shoulder_link | 1993.14 |
| J2 / upper_arm_link | 1332.24 |
| J3 / forearm_link | 685.11 |
| J4 / wrist_1_link | 579.56 |
| J5 / wrist_2_link | 523.81 |

## Applied tool masses

| Tool | Fixed assembly, g | Left moving assembly, g | Right moving assembly, g | Total, g |
| --- | ---: | ---: | ---: | ---: |
| Flange | 55.5 | n/a | n/a | 55.5 |
| MSG / Solid jaws or Soft TPU jaws | 301.4 | 39.3 | 39.3 | 380 |
| SSG48 / Solid jaws or Fin-ray jaws | 361.4 | 39.3 | 39.3 | 440 |

A moving assembly includes its slider/carriage, finger, jaw and included
fasteners. These are user-reported measurements; the 39.3 g figure was given
approximately. Parameter formatting does not imply additional measurement
precision. No mass was inferred from collision mesh volume for these bodies.

## Properties still estimated

Only the tool masses changed. Each format's existing gripper/flange COM and
central inertia values are retained numerically as unverified estimates.
They were not rescaled from the new masses, fitted to force agreement, or
replaced with the flange's zero-inertia runtime idealization. Their CAD mass
distribution still needs checking. Equal mass across jaw options does not
establish equal real COM or inertia. Simulated tool dynamics remain approximate.

Motor rotor/armature estimates are unchanged, as requested. No RCB rotor value
has been adopted as a calibrated measurement. The fixed base is also unchanged.

The working RCB runtime files are read-only for this exporter. Its tool XML
still contains the older configured MSG/SSG48 totals. The audit identifies
those differences; this package update does not deploy or retune hardware.

## Validation

All five profiles were checked at 160 poses each,
including limit corners and random samples across the RCB software ranges.
The largest arm COM difference was 0.003821 mm. The isolated five-link arm
comparison, excluding both tools and motor rotors, found a maximum gravity
torque difference of 0.00003741 Nm and mass-matrix relative difference of
0.001512%. Residual numerical differences reflect existing geometric
rounding. These are model consistency checks, not physical measurement accuracy.

The comparison also confirms geometry/assets, kinematics, collisions, joint
limits, drive gains, armature and fixed-base properties are unchanged. Tool
COM and central tensors are unchanged. Both jaw regeneration paths retain the
synchronized parameters. Repeated synchronization is byte-identical.

All eight rigid-grasp fixtures were regenerated with the measured component
masses. Seven meet the stricter hold/release target; SSG48 Solid jaws retain
about 31.6 mm of the commanded 50 mm lift on the box, below the 35 mm target.
This uncalibrated contact test is separate from the arm-property validation.

The physical drive hold/jog regression tests pass with their existing gains,
and both SSG48 jaw profiles pass the full-arm gravity-hold check. A total of
69 selected automated checks pass, including mapping, jaw switching, contact
fixtures, mass partition and tensor conversion.

## Rebuilding in the source repository

From `robot description/PAR6 Description`, run:

```bash
python tools/sync_model_dynamics.py
python tools/audit_rcb_dynamics.py
python tools/package_models.py
```

The first command also regenerates grasp scenes. The build requires the local
RCB checkout and numpy, scipy, roboticstoolbox-python and mujoco; distributing
the exported models does not require those source tools.

Inputs are `source/dynamics/model_mapping.json` and
`source/dynamics/physical_measurements.json`, with read-only RCB arm data.
`source/dynamics/rcb_arm_reference.generated.json` records source hashes,
resolved frame conversions and exported central tensors; regenerate it rather
than editing it. Jaw builders invoke the same synchronization after rebuilding.
See `reports/dynamics_sync_validation.json` in the source repository for the
per-profile results. `DYNAMICS_AUDIT.md` compares the whole configured models;
its remaining tool/rotor differences are expected pending their calibration.
