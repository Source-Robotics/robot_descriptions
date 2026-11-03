# PAROL6 common-frame validation

Release 2.9.1, Windows x64 / CPython 3.11, 2026-10-31.

- PAROL6 public joint axes and origin orientations now match PAR6 in both
  formats, including J6. No public joint-angle sign conversion is required.
- Only standard-DH d4 differs: PAR6 -177.00 mm, PAROL6 -176.35 mm. At the same
  joint coordinates, all five profiles show 0.650000 mm flange translation
  difference and zero orientation difference against the matching PAR6 format.
- Across 160 configurations per profile, the maximum residual after the
  forearm translation is accounted for is 5.1e-12 mm / 0 degrees.
- Local masses, COM, central inertia, armature, damping, friction and actuator
  gain/bias/torque-limit arrays match PAR6. Native tool subtrees are unchanged.
- Historical source bounds are converted to the normalized directions;
  elbow bounds change from [-2, 1.3] to [-1.3, 2] radians. Original input files
  and mesh triangle records are retained unchanged. Source rounded joint
  angles are replaced in generated files by PAR6's reference orientations.
- 33 standalone description tests passed. The full independent build passed,
  including all four release archives, five variants per archive and sixteen
  self-contained native grasp-scene compilations.
- Repeated the full build from a fresh local Git clone with Windows
  `core.autocrlf=true`, isolated Python, and a working directory outside the
  checkout. The build and all 33 tests passed; `git diff --exit-code` was clean.
  All four archives matched the original checkout byte-for-byte.
- All 69 selected existing motion-application tests passed. All five compiled
  PAR6 scenes matched their recorded physical/geometry baseline exactly.
- The user selected finite position control with torque limits. Gains were
  retained as existing simulation estimates, with no infinite-stiffness or
  hardware-calibration claim. Five three-second dynamics smoke checks passed
  without numerical warnings, nonfinite states or actuator torque-limit
  violations; details are in `parol6_actuation_smoke.json`.

The original export-to-new-package sign conversion is
[1, 1, -1, -1, -1, -1]; it applies to saved trajectories from the prior local
2.9.0 convention. New PAR6/PAROL6 package coordinates match directly.

Native and URDF TCP frames agree within 0.002299 mm / 0.000933 degrees;
the existing native PAR6 quaternion precision is inherited. These are
representation precision differences, not extra DH geometry differences.

The shared 57.1 mm flange distance remains the requested PAR6 interface.
Controller calibration, physical stepper stall/lost-step behavior and a real
PAROL6 encoder/homing mapping are not established by these software checks.
See `ACTUATION.md` and `KINEMATICS_COMPARISON.md` in each portable package.

## Release archive SHA-256

```text
02b040248bdb1c29361ab8dd3bbc98a60eda94a078a4cb02c2e5e1ac4290e906  par6_mujoco.zip
9e033dd73c29c303041946b7e30fafb49084715fcd23f838e984cea159b79733  par6_urdf.zip
66133c0604abde09e91f1747d55185395151f53611964ca4bf1f489adc3d43e0  parol6_mujoco.zip
3fb0e15cde4531669192c47157d192a4d71d07e20f9725c58bb767fc5f27a26c  parol6_urdf.zip
```

The Linux GitHub workflow is prepared but has not been executed on GitHub.
