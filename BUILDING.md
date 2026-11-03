# Building from an independent checkout

This folder is the repository root. It can live anywhere, including inside
another repository as a Git submodule. The normal build needs no `IK`,
`par6_motion`, RCB-Runtime, robotics-toolbox installation, or sibling checkout.
All source models, meshes, STEP files, artwork and reference data are contained
here. Exported packages remain independent of the authoring tools.

All maintained appearance settings are in
[`source/appearance/colors.json`](source/appearance/colors.json).
For examples and individual-part overrides, see [Changing model colors](docs/COLORS.md).

## Setup

These are the CAD/model-authoring dependencies. To use a committed/downloaded
URDF in ROS 2, follow [ROS2.md](docs/ROS2.md) instead: it needs ROS dependencies
and colcon, without this authoring virtual environment.

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

Every runtime package keeps its model/scene entry files, README, license and
metadata at the root. Detailed notes and dimensioned sheets are generated under
`docs/`; catalog PNGs are under `previews/`. The generators write those locations
directly. Packaging checks local documentation and catalog links again after
extracting each ZIP, so the organized downloads remain portable.

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
   Shared palette application changes only visual colors, with an XML guard
   against changing physical definitions; both arms and extracted tools use
   the same palette. Each exported package includes `docs/colors.json`.
9. Regenerates twelve dimensioned kinematic review sheets and validates each
   six-row DH table against both formats at 128 poses. Uses checked snapshot
   caches, without requiring OpenGL during a normal build.
10. Exports independent MSG/SSG48 packages with both jaw options, compares
   their tool-relative FK, inertials, controls and contacts against the mounted
   models, and runs finite close/open cycles.
11. Writes and extracts all eight ZIPs: `par6_mujoco.zip`, `par6_urdf.zip`,
   `parol6_mujoco.zip`, `parol6_urdf.zip`, `msg_mujoco.zip`, `msg_urdf.zip`,
   `ssg48_mujoco.zip` and `ssg48_urdf.zip`. Each uses only its own assets.

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

After editing the shared color file, refresh review snapshots during the same
build (requires working OpenGL):

```bash
python tools/build.py --models-only --render
```

This also rebakes badge colors, without retessellating STEP geometry. The
ordinary build and CI reuse the committed snapshots without OpenGL. Separate
guide/gallery screenshots can be refreshed with the optional renderers listed
in the color guide.

The entry point also supports isolated Python mode and invocation by absolute
path from another working directory: `python -I /path/to/checkout/tools/build.py`.

The build also generates native ROS 2 manifests, CMake installation, display
launchers, RViz configuration and portable in-memory asset helpers for all four
URDF packages, retaining conditional ROS 1 support. Linux CI tests Humble and
Jazzy installation, TF, mimic joints and the display applications separately
from the CAD authoring environment.

## Reference data and explicit refresh

`source/dynamics/rcb_reference.json` is a versioned snapshot captured from the
preferred RCB model. It contains standard-DH geometry, software limits, arm
mass properties and the captured tool/rotor estimates, with upstream hashes.
`model_reference.py` evaluates that data using NumPy. Regression fixtures in
`rcb_reference_checks.json` were recorded from the live runtime and check FK,
gravity and mass matrices without importing it.

`physical_measurements.json` supplies the corrected tool masses. Motor rotors
remain uncalibrated estimates; tool COM/tensors and fixed-base properties have
the same status documented in `docs/DYNAMICS.md`.

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
Read `docs/ACTUATION.md` in each package for the existing finite position-control
estimates. Open-loop hardware does not imply infinite simulation stiffness;
URDF controller gains are configured in the consuming simulator/controller.

## Additional tools

### Dimensioned kinematic review

All four packages contain `docs/KINEMATIC_REVIEW.md` and six sheets under
`docs/kinematics_review/`, in SVG and PNG with companion JSON. The authoring
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

MSG built-in camera examples use the existing box grasp fixtures. Refresh them
with `python tools/render_msg_camera_previews.py` on a machine with OpenGL;
the normal build copies the committed PNGs and `docs/MSG_CAMERA.md` into the
six arm/MSG packages. Camera pose/FOV and model definitions are retained.
See [the camera guide](docs/MSG_CAMERA.md) for scene and lighting details.

Standalone gripper guide images can be refreshed separately on a machine with
OpenGL, for example `python tools/render_gripper_previews.py --gripper msg`.
The MSG preview uses a temporary presentation pose with the lens upright and
jaws facing the viewer. It does not change the model's mounting frame or physics.
The next standalone export copies these images into both package formats;
`tools/export_hardware_guide.py` copies them into the hardware guide.

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

## Hardware guides and software integration

Keep model maintenance here. The hardware repositories contain ordinary
`par6_description/`, `parol6_description/`, `msg_description/` or
`ssg48_description/` guide folders, with README, previews and release-download
links. They do not include model copies or submodules. See `RELEASING.md` for
`tools/export_hardware_guide.py`, which regenerates those guide folders.

Software projects that load models directly from a checkout may include this
repository as a submodule, for example `external/robot_descriptions`. Pin a
tested commit or release and select the product's package paths within it.
A submodule includes the whole central repository. Users of those software
projects can initialize it with `git submodule update --init --recursive`.
