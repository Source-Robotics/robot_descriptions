# PAR6 and PAROL6 release validation

Version 2.9.0, Windows x64 / CPython 3.11, 2026-10-31.

- Full build and validation completed from an isolated environment with no
  system site-packages, RCB runtime, robotics-toolbox or motion application.
- Repeated the full build from a fresh local Git clone with
  `core.autocrlf=true`, invoked with isolated Python from outside the checkout.
  The clone build passed, `git diff --exit-code` found no tracked content
  changes, and all four archives were byte-for-byte identical to the original
  checkout's archives.
- 32 description regression tests passed.
- All 69 selected existing motion-application regression tests passed.
- Five PAROL6 profiles checked at 160 configurations each against the supplied
  source URDF; arm joint origins, axes, types, limits and FK are preserved.
- Corresponding PAR6 mass/COM/central inertia and armature estimates are
  inherited and checked for each profile. Complete native tool subtrees are
  unchanged; original PAROL6 collision meshes and visual triangle records
  are preserved.
- All four self-contained release archives pass asset/CRC and variant checks;
  all sixteen packaged native grasp scenes compile.
- Five PAROL6 profile previews and the frame diagram were rendered and
  visually inspected.
- The original supplied PAROL6 input hashes are unchanged. A compiled
  before/after check of all five existing PAR6 scenes found a maximum
  numerical difference of 0.0 in the recorded physical/geometry parameters.

The 32 description tests also passed from the fresh Git clone.

## Release archive SHA-256

```text
415868029a2377d9038c354959ce9fffcfa15bd4d4843f3f2f599365f147a529  par6_mujoco.zip
786766cb2d42410e013397704fad56f79b883bd5e3d1c2181f5f63b42719e5df  par6_urdf.zip
45f31b88ce60bc8ba8dbe8a2a6c195ac8cf55ee03b7eaecde20d5574d74d44af  parol6_mujoco.zip
b82629d3bbd6496e8192f2241f1a3e2aecf698b865a2488df8fc3c7b1b9b11a8  parol6_urdf.zip
```

The supplied PAROL6 forearm is 0.65 mm shorter than PAR6. Matched-direction
flanges differ by up to 0.650707 mm / 0.000636 degrees over the sampled poses;
the orientation residue is consistent with the retained rounded RPY values.
The common flange offset is inherited from PAR6 under the shared-tool
assumption; no independent flange marker exists in the supplied export.
Hardware encoder/homing directions and measured PAROL6 inertias were not
established by these software checks.

The Linux GitHub Actions workflow is prepared; no GitHub run is claimed.
The historical strict SSG48 Solid jaws grasp-fixture limitation is unchanged.
