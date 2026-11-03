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

All eight rigid-grasp fixtures per robot pass the hold/release targets after
correcting initial object/housing overlap for the shorter Solid jaws. That
50 g box now retains about 37.3 mm of the commanded 50 mm lift. Internal
SSG48 URDF rack collision proxies were corrected separately; visuals,
per-link masses/COM/tensors, drive settings, friction and pass thresholds
are retained. See `SSG48_CONTACTS.md` for scope and reproduction. The current
standalone tests also cover both robot families, both formats, jaw travel,
contact fixtures, mass partition and tensor conversion.

## Rebuilding in the source repository

From `robot description/PAR6 Description`, run:

```bash
python tools/sync_model_dynamics.py
python tools/audit_rcb_dynamics.py
python tools/package_models.py
```

The first command also regenerates grasp scenes. Version 2.8.1 builds from the checked-in RCB reference snapshot, without a
local runtime, motion application or robotics-toolbox installation. Use
`python tools/build.py` in the source repository; see its `BUILDING.md` for
the locked environment and the separate explicit reference-refresh command.

Inputs are `source/dynamics/model_mapping.json` and
`source/dynamics/physical_measurements.json`, with the versioned `source/dynamics/rcb_reference.json` snapshot.
`source/dynamics/rcb_arm_reference.generated.json` records source hashes,
resolved frame conversions and exported central tensors; regenerate it rather
than editing it. Jaw builders invoke the same synchronization after rebuilding.
See `reports/dynamics/dynamics_sync_validation.json` in the source repository for the
per-profile results. `DYNAMICS_AUDIT.md` compares the whole configured models;
its remaining tool/rotor differences are expected pending their calibration.

## Pneumatic profile estimates

The sixth pneumatic profile uses the legacy, unverified 195 g total. Body/jaw masses are distributed by CAD volume with local COM and central tensors computed from those solids. It is separate from the five profiles with user-reported tool mass measurements. See [PNEUMATIC.md](PNEUMATIC.md) for the source, motion, collision and finite actuator assumptions.
