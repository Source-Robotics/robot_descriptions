# Standalone authoring validation

Validated on 2026-10-31 for description version 2.8.1, using Windows x64,
CPython 3.11 and the pinned build environment. The virtual environment had
system site-packages disabled and contained neither RCB-Runtime nor the
motion application.

| Check | Result |
| --- | --- |
| Full build from a fresh local Git clone of the description folder alone | Passed |
| Clone with `core.autocrlf=true`, invoked from outside the checkout with `python -I -B` | Passed |
| Standalone regression suite | 20 passed |
| Existing selected motion-application regression suite | 69 passed |
| Generated tracked content after the clone build | `git diff --exit-code` passed |
| MuJoCo and URDF archives compared with the original checkout | Byte-for-byte identical |
| Release archive integrity, assets and all five variants | Passed |
| Self-contained MuJoCo grasp fixtures compiled | 8 passed |
| Dynamics validation against the captured reference | 160 poses per profile, all five profiles passed |
| Portable NumPy reference compared with the live RCB model | 120 random configurations passed for FK, mass matrix and gravity |
| Captured upstream RCB source hashes after this work | All six files unchanged |

The model preservation check compared all five compiled native scenes with a
baseline captured before this portability work: masses, COM, reconstructed
central inertia tensors, body transforms, armature, joint limits, collision
placement/size/masks and actuator gain/bias parameters. The maximum numerical
difference was **0.0**. Calibration status and existing estimates are unchanged.

## Reproducible release outputs

SHA-256 values from both checkouts:

```text
fdae6df793db8edb150ba3fc3e6411303a00cc2cc4ea22c4a5251da81eab5b7d  par6_mujoco.zip
f5c6e29538a42ed51fb04f9a6e8b4431e9a898097c892dcf8805d3fd290fe358  par6_urdf.zip
```

The build uses a versioned RCB snapshot and included CAD/artwork inputs;
refreshing the upstream reference is a separate explicit operation. See
[BUILDING.md](../../docs/maintenance/BUILDING.md) for setup and commands.

The Linux GitHub Actions workflow is prepared but has not been run on GitHub.
This validation establishes reproducibility in the tested Windows environment;
it does not claim bitwise equality across different operating systems or CAD
library versions. The previously documented strict SSG48 Solid jaws grasp
fixture limitation remains unchanged.
