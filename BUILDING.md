# Building from an independent checkout

This folder is the repository root. It can live anywhere, including inside
another repository as a Git submodule. The normal build needs no `IK`,
`par6_motion`, RCB-Runtime, robotics-toolbox installation, or sibling checkout.
All source models, meshes, STEP files, artwork and reference data are contained
here. Exported packages remain independent of the authoring tools.

## Setup

Use 64-bit CPython 3.11 and an isolated virtual environment:

```bash
python -m venv .venv
```

Activate with `.venv\Scripts\Activate.ps1` in Windows PowerShell, or
`source .venv/bin/activate` on Linux/macOS, then:

```bash
python -m pip install -r requirements-build.lock.txt
python -m pytest tests -q -p no:cacheprovider
python tools/build.py
```

`requirements-build.txt` lists direct dependencies. The lock file also pins
transitive dependencies from the tested environment. Update these pins
intentionally and repeat the build tests. Native dependencies still apply:
Gmsh needs its platform libraries, and CairoSVG needs Cairo. On Ubuntu the
CI job installs `libglu1-mesa`, `libcairo2` and `libegl1`. Windows users need a
working Cairo library installation discoverable by CairoSVG; see
https://cairosvg.org/documentation/ for its platform installation guidance.

## What the build does

1. Regenerates visual partitions from the preserved original STL records.
2. Bakes the editable SVG palette onto the cover texture.
   Reuses a checked SVG/configuration/generator-keyed raster cache to keep
   Cairo antialiasing differences from changing unchanged artwork across OSes.
3. Verifies/tessellates the STEP jaw sources and rebuilds the selected jaws.
   Checked tessellations include the STEP hash, solid IDs, meshing settings,
   Gmsh version and generator hash, preserving identical reviewed surfaces
   across platforms until those authoring inputs change.
4. Builds six convex contact pieces per finger, reusing validated cached pieces
   only when source hashes match.
5. Applies the captured arm reference and user-reported tool masses.
6. Builds the pneumatic profile from the original L6 assembly, with cached
   convex contacts, two coupled jaws and explicitly estimated physical data;
   rebuilds eight electric-gripper grasp fixtures and validates measured data.
7. Builds six PAROL6 variants from the included original export, reusing the
   PAR6 tool profiles, physical data, axes and origin orientations. Both use
   a 177 mm forearm. PAROL6 lower-arm geometry moves outward 0.65 mm from the
   export; its 50.5 mm flange reach matches PAR6 housing-to-tool spacing.
8. Validates all PAROL6 profiles against the supplied source chain; writes
   the shared DH/frame comparison and diagram into all four packages.
9. Regenerates twelve dimensioned kinematic review sheets and validates each
   six-row DH table against both formats at 128 poses. Uses checked snapshot
   caches, without requiring OpenGL during a normal build.
10. Writes and checks `dist/par6_mujoco.zip`, `dist/par6_urdf.zip`,
   `dist/parol6_mujoco.zip` and `dist/parol6_urdf.zip`.

Byte-hashed STEP/STL/SVG inputs are protected from Git line-ending conversion
by `.gitattributes`. Reference snapshot inputs use LF. Release ZIPs use fixed
timestamps and canonical LF for text assets so checkout timestamps and Windows
text conversion do not change otherwise identical archive contents.

The tracked XML/URDF model definitions are editable inputs; the build updates
their generated assets and inertial fields. It does not reconstruct all
robot structure from an unavailable SolidWorks assembly. The required
historical pinch meshes and STEP files are included under `source/`.

For a faster build using committed CAD/visual assets:

```bash
python tools/build.py --models-only
```

The entry point also supports isolated Python mode and invocation by absolute
path from another working directory: `python -I /path/to/checkout/tools/build.py`.

## Reference data and explicit refresh

`source/dynamics/rcb_reference.json` is a versioned snapshot captured from the
preferred RCB model. It contains standard-DH geometry, software limits, arm
mass properties and the captured tool/rotor estimates, with upstream hashes.
`model_reference.py` evaluates that data using NumPy. Regression fixtures in
`rcb_reference_checks.json` were recorded from the live runtime and check FK,
gravity and mass matrices without importing it.

`physical_measurements.json` supplies the corrected tool masses. Motor rotors
remain uncalibrated estimates; tool COM/tensors and fixed-base properties have
the same status documented in `DYNAMICS.md`.

Refreshing the snapshot is a separate maintainer operation, not part of build:

```bash
python tools/import_rcb_reference.py --rcb-root /path/to/RCB-Runtime --date YYYY-MM-DD
```

That command alone needs the upstream runtime and its dependencies. It reads
the named checkout and never edits it. Review the snapshot and fixture diffs,
then rerun tests and the build. Normal builds never search for a runtime or
silently update the reference. Do not hand-edit generated
`rcb_arm_reference.generated.json`; the build recreates it from the snapshot,
measured masses, mapping and existing model frames.

## PAROL6 source and shared model policy

`source/parol6/original` contains the supplied URDF and STL export;
`source/parol6/model.json` records hashes, visual component assignments and
the inheritance policy. Normal builds do not read a sibling `PAROL6_URDF`.
To replace those inputs deliberately, use:

```bash
python tools/build_parol6.py --import-source /path/to/exported/parol6
```

Review component assignments if meshes change; the existing assignments are
guarded by source hashes. Run the full build after reviewing the import.
The original export is preserved byte-for-byte as source. Generated models
use PAR6 axes and joint-origin orientations, including J6, and a corrected
177 mm forearm. `source/parol6/model.json` records the lower-arm origin shift
and the shorter 50.5 mm flange reference, derived from the CAD housing faces.
Every flange-relative tool subtree is reused without editing its internals.
Source joint bounds are converted into these coordinates.
Mass/COM/tensors and end effectors are inherited from the corresponding PAR6
format as requested. This includes existing calibration limitations.

`python tools/verify_parol6.py` checks all profiles across 160 poses each and
regenerates the numeric frame/DH documentation. Preview PNGs can be refreshed
separately with `python tools/render_parol6_previews.py` on a machine with a
working OpenGL backend, followed by a build to update preview metadata.
The ordinary build and CI do not require offscreen rendering.
Read `ACTUATION.md` in each package for the existing finite position-control
estimates. Open-loop hardware does not imply infinite simulation stiffness;
URDF controller gains are configured in the consuming simulator/controller.

## Additional tools

### Dimensioned kinematic review

All four packages contain `KINEMATIC_REVIEW.md` and six sheets under
`kinematics_review/`, in SVG and PNG with companion JSON. The authoring
checkout also contains an offline comparison page at
`reports/kinematics_review/index.html`. All diagrams use the same zero-angle
pose and orthographic camera. Dimensions are extracted from URDF joint origins
and fixed TCP transforms; the snapshot uses native MuJoCo visual meshes.

Normal builds validate the committed source snapshots in
`source/reviews/render_cache/`. They fail with a refresh instruction if geometry,
materials or camera settings change. To refresh them on a machine with OpenGL:

```bash
python tools/build_review_sheets.py --render
```

To generate the optional printable twelve-page A4 booklet:

```bash
python -m pip install -r requirements-review.txt
python tools/build_review_sheets.py --pdf
```

The booklet is written to `output/pdf/robot_kinematics_review.pdf`. Rendered robot
images are embedded; dimension arrows and tables remain vector graphics in SVG
and PDF. The optional PDF dependency is not required by ordinary builds or CI.
Changing a jaw does not automatically measure a new contact/grasp frame; the
diagrams explicitly show the retained operational TCPs.

### Other checks

```bash
python tools/verify_dynamics_sync.py
python tools/audit_rcb_dynamics.py
python tools/grasp_check.py --all
python tools/benchmark_visuals.py --rounds 1 --frames-per-round 1
```

All eight grasp cases pass in each robot package. Add `--robot parol6` for
that package's fixtures. The full build runs all 16 grasps and eight 201-point
SSG48 internal contact sweeps before packaging. Invalid initial object
penetration is rejected. See `reports/ssg48_contact_diagnosis.md` for the rack
collision and short-jaw fixture corrections. Force/friction/compliance remain
uncalibrated. Viewing/rendering needs a working OpenGL backend; the core build
does not open a viewer.

Dynamics validation works without an external baseline. For before/after
geometry and drive preservation checks, optionally supply both
`--baseline-mujoco PATH --baseline-urdf PATH`. Historical appearance/contact
verification commands also require explicitly supplied comparison packages;
those archived baselines are not required to build a clean checkout.

## PAR6 main-repository integration

Keep model maintenance here and add this repository as a submodule such as
`models/descriptions` in the PAR6 main repository. Pin a tested commit; a model
update becomes a deliberate submodule-pointer change in a normal PR. A
submodule references an entire repository, so a central descriptions repo
includes all its products even when PAR6 uses only its own package paths.

Users cloning the main repository use `git clone --recurse-submodules URL`,
or initialize an existing clone with `git submodule update --init --recursive`.
Keep ready-to-use release ZIP links in the product README as well.
