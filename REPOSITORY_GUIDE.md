# What is in this repository?

**To use a model, you only need one package from `mujoco/` or `urdf/`, or its
release ZIP.** The rest supports editing, rebuilding, checking and publishing
those packages. Start with your product's [getting-started guide](guides/).

This guide covers every top-level folder and file, then the contents of the
packages, source inputs, tools and reports. Repeated meshes and configuration
variants are grouped by purpose rather than listing the same description for
hundreds of individual STL files.

## The top-level folders

| Folder | What it contains | When you need it |
| --- | --- | --- |
| [`guides/`](guides/) | Short instructions for each product, including downloads and which model to open. | Start here. |
| [`mujoco/`](mujoco/) | Four independent native MuJoCo packages: `par6/`, `parol6/`, `msg/`, `ssg48/`. | Using MuJoCo. |
| [`urdf/`](urdf/) | Four independent URDF/ROS packages: `par6_description/`, `parol6_description/`, `msg_description/`, `ssg48_description/`. | Using URDF, RViz or another URDF consumer. |
| [`moveit/`](moveit/) | `par6_moveit_config/` and `parol6_moveit_config/`: planning groups, TCP, joint limits, SRDF collision rules, OMPL and mock execution. Each depends on its matching URDF package. | Planning and previewing trajectories in ROS 2. |
| [`docs/`](docs/) | Color/camera guides in `COLORS.md` / `MSG_CAMERA.md` and images under `images/`. | Browsing illustrations. |
| [`LICENSES/`](LICENSES/) | Apache-2.0 software license text. | Maintaining or redistributing support code. |
| [`source/`](source/) | Original CAD exports, STEP jaws, editable logo, measured data, reference snapshots and checked rendering/geometry caches. | Changing or rebuilding models. |
| [`tools/`](tools/) | Python generators, validators and packaging tools, plus geometry/benchmark records. | Maintaining the models. |
| [`tests/`](tests/) | Automated regression tests for the generators and model data. | Checking a change. |
| [`reports/`](reports/) | Dynamics comparisons, collision/grasp results, kinematic diagrams and build records. | Reviewing accuracy or investigating a result. |
| [`output/`](output/) | The printable dimensioned review booklet, under `pdf/`. | Comparing robot geometry visually. |
| [`releases/`](releases/) | Markdown release notes for published versions. The downloadable ZIPs live on GitHub Releases. | Reviewing release changes. |
| [`.github/`](.github/) | `workflows/models.yml`: automated tests, builds and tagged-release publishing. | Maintaining GitHub automation. |
| `dist/` | Locally generated ZIPs, release PDF, manifest and checksums. Ignored by Git. | Building a release locally. |

`.git/` is local Git history and metadata, not robot data. A local `.venv/`,
`venv/`, `__pycache__/` or `.pytest_cache/` is a Python environment/cache, not a
model input; these are also ignored by Git.

## Files at the repository root

| File | Purpose |
| --- | --- |
| [`README.md`](README.md) | Public front page, package downloads and product links. |
| [`REPOSITORY_GUIDE.md`](REPOSITORY_GUIDE.md) | This explanation of the repository. |
| [`BUILDING.md`](BUILDING.md) | How to install authoring dependencies, regenerate models and run checks. |
| [`RELEASING.md`](RELEASING.md) | How to publish a version and update the hardware-repository guides. |
| [`CHANGELOG.md`](CHANGELOG.md) | Version-by-version changes. |
| [`MODEL_DETAILS.md`](MODEL_DETAILS.md) | Longer model/authoring notes, including earlier visual and jaw changes. Some sections describe historical work; use current product guides and package documentation for current entry files. |
| [`ATTRIBUTION.md`](ATTRIBUTION.md) | Source Robotics model credit, license scope and prior-release history. |
| [`LICENSE`](LICENSE) | CC BY 4.0 license text for the models and assets. |
| [`requirements-build.txt`](requirements-build.txt) | Direct Python dependencies used by the authoring tools. |
| [`requirements-build.lock.txt`](requirements-build.lock.txt) | Pinned, tested dependency versions, including transitive dependencies. Use this for a reproducible build. |
| [`requirements-review.txt`](requirements-review.txt) | Additional dependencies for generating the optional review PDF. |
| [`.gitattributes`](.gitattributes) | Protects hash-sensitive CAD/mesh inputs and controls Git line endings. |
| [`.gitignore`](.gitignore) | Excludes local environments, caches, logs and generated `dist/` files. |

The former loose PNGs now live in `docs/images/`: `motor_visuals_preview.png`,
`gripper_visuals_preview.png`, `msg_jaws_preview.png`, `ssg48_jaws_preview.png`
and `side_badge_preview.png`. They illustrate feature work; they are not simulator
textures. The same folder contains the PAR6/PAROL6 getting-started guide images.

## What the file extensions mean

| Extension | Meaning here |
| --- | --- |
| `.urdf` | Robot links, joints, visual/collision geometry and inertia for a URDF consumer. |
| `.xml` | Usually native MuJoCo data inside `mujoco/`; ROS manifests and launch configuration use XML too, so the path/name matters. |
| `.STL` | Triangle mesh geometry. STL has no material/color assignment; those are set by the model. |
| `.obj`, `.mtl`, `.dae` | Visual mesh/material formats used for textured geometry, including the badge. |
| `.STEP` | Original editable CAD solids used to generate jaw meshes. |
| `.svg` | Vector artwork or dimensioned drawings. |
| `.png` | Preview/snapshot image, or a texture when referenced from a material. |
| `.json` | Configuration, metadata, cached geometry or validation results, depending on the folder. |
| `.py` | Python generator, helper, validator or test. |
| `.md` | Human-readable documentation. |
| `.yaml`, `.rviz`, `.launch`, `.launch.py` | ROS configuration, saved RViz views and ROS 1/2 launch files. |
| `.csv` | Original CAD/URDF export tables under `source/legacy/`. |
| `.pdf` | Printable review/inertial-data documents. |

## Product guides

| File | Purpose |
| --- | --- |
| [`guides/par6.md`](guides/par6.md) | PAR6 downloads, six tool choices and opening instructions. |
| [`guides/parol6.md`](guides/parol6.md) | PAROL6 downloads, six tool choices and opening instructions. |
| [`guides/msg.md`](guides/msg.md) | MSG alone or arm-mounted, Solid/Soft TPU selection and control. |
| [`guides/ssg48.md`](guides/ssg48.md) | SSG48 alone or arm-mounted, Solid/Fin-ray selection and control. |
| [`guides/legacy-models.md`](guides/legacy-models.md) | Migration from the older model folders in the hardware repositories. |

Hardware repositories have their own ordinary `par6_description/`,
`parol6_description/`, `msg_description/` or `ssg48_description/` guide folder.
Those contain a README and previews. The folders under **this repository's
`urdf/`** contain the actual URDF packages; the names intentionally match the
product convention, but their contents serve different purposes.

## Inside a MuJoCo package

`mujoco/par6/` and `mujoco/parol6/` each contain the complete arm and six tool
configurations. `mujoco/msg/` and `mujoco/ssg48/` contain only that gripper and
its two jaw options. They have a fixed mounting frame and moving fingers.

| File or folder | Purpose |
| --- | --- |
| `scene.xml` | Default viewer scene: MSG for either arm; Solid jaws for either standalone gripper. |
| `scene_<option>.xml` | Viewer scene for one option; includes the model and adds a floor, lighting and viewer settings. Open this for a quick look. |
| `<product>_<option>.xml` | Native MJCF robot/gripper definition: bodies, joints, inertials, visuals, collisions and actuators. Use this when integrating into another MuJoCo world. |
| `assets/` | Meshes and textures referenced by that package. Keep them with the XML files. |
| `assets/visual/` | Mesh partitions that let motors, gearboxes, lens and structural parts use different materials. |
| `assets/branding/` | Textured side-panel visuals, UV meshes/materials and the baked badge PNG. |
| `assets/jaws/` | Alternate-jaw visual geometry and associated meshes. |
| `assets/contacts/` | Small convex collision pieces for the electric-gripper fingers. These approximate contact surfaces; they are not extra physical links. |
| `assets/pneumatic/` | Pneumatic housing/jaw visuals and convex contacts in the arm packages. |
| `bench/` | Arm-package grasp-test scenes: four electric-jaw profiles, each with a box and cylinder. These are movable test fixtures, not full arm models or the public standalone-gripper packages. |
| `bench/profiles.json` | Grasp-fixture settings, object dimensions and test positions. |
| `bench/README.md` | How the fixtures and grasp checks work. |
| `sim2bot.robot.json` | Optional package/variant index: entry paths, previews, actuator names and TCP names. MuJoCo itself loads XML without this file. |
| `previews/<product>_<option>.png` | A model preview for documentation/catalog viewers. Not a texture. |
| `docs/kinematics_comparison.svg` | PAR6/PAROL6 frame and dimension comparison, included with arm packages. |
| `docs/kinematics_review/` | Six arm/tool review sheets; see below. |

Not every package needs every asset subfolder. Each contains its own required
assets so it can be moved or downloaded independently. Meshes repeated across
packages support that independence; each model only loads its referenced files.

Arm option suffixes are `flange`, `msg`, `msg_tpu_glue`, `ssg48`,
`ssg48_stock` and `pneumatic`. The older suffixes remain for compatibility:
`msg_tpu_glue` means **MSG Soft TPU**, and `ssg48_stock` means **SSG48 Fin-ray**.
Standalone suffixes are `solid` / `soft_tpu` for MSG and `solid` / `fin_ray` for
SSG48.

Detailed notes are grouped in `docs/`, which has its own clickable index.
Only the README, license, package metadata and model/scene entry files stay at
the root. This layout is shared by all eight runtime packages.

## Inside a URDF package

The four folders are `urdf/par6_description/`, `urdf/parol6_description/`,
`urdf/msg_description/` and `urdf/ssg48_description/`.

| File or folder | Purpose |
| --- | --- |
| `<product>_<option>.urdf` | The selected robot/gripper: links, joints, limits, inertials, visual meshes and collision meshes. Open this in a URDF consumer. |
| `meshes/` | All geometry and textures used by the package, grouped by arm/tool and purpose. `visual/`, `branding/`, `jaws/`, `contacts/` and `pneumatic/` serve the same roles as their MuJoCo counterparts, sometimes nested under a tool folder. |
| `package.xml` | ROS package identity, version, license and dependencies. This XML is not a MuJoCo model. |
| `CMakeLists.txt` | Native ament/colcon installation for ROS 2; conditional catkin path for a sourced ROS 1 environment. |
| `launch/display.launch.py` | ROS 2 display with tool/jaw selection, joint sliders and saved RViz configuration. |
| `launch/display.launch` | ROS 1 launch to display the selected configuration in RViz. |
| `launch/gazebo.launch` | Existing ROS 1/Gazebo launch scaffold in arm packages; simulator/controller setup remains the consumer's responsibility. |
| `config/joint_names.yaml` | Arm joint-name configuration. |
| `config/display.rviz` | Saved ROS 2 RViz view with the product's fixed frame, visual model and scale. |
| `scripts/robot_description.py` | Shared model selection and in-memory ROS 2 `package://` resource resolution. |
| `scripts/ros_urdf.py` | ROS 1 helper for portable `package://` paths. It is not a controller. |
| `sim2bot.robot.json`, previews and review drawings | Package index and documentation, as above. |

URDF describes the mechanism and its physical data. It does not by itself
provide a motor controller; gains and simulator-specific behavior belong in
the consuming software. Native ROS 2 installation/display and retained ROS 1
entry points are explained in [the ROS 2 guide](docs/ROS2.md).

## Documentation bundled inside packages

These copies let a downloaded ZIP carry its own operating and calibration
notes. Most appear in all four arm/format packages; standalone grippers have
a root README/LICENSE and `docs/VALIDATION.md`.

| File | What it explains |
| --- | --- |
| `README.md` | How to open the package and select a configuration. |
| `LICENSE` | License carried with the exported package. |
| `docs/ATTRIBUTION.md` | Source Robotics credit and CC BY 4.0 model-license scope; URDF packages also include `docs/LICENSE-CODE.txt` for Apache-2.0 support code. |
| `docs/MSG_CAMERA.md` | Simulated built-in MSG camera views, FOV/frame information and a Python rendering example; included in arm/MSG packages. |
| `docs/COLORS.md`, `docs/colors.json` | Shared appearance instructions and the applied palette snapshot carried by each package. |
| `docs/ROS2.md` | Installation and ROS 2 display instructions, included in all four URDF packages. |
| `docs/README.md` | Clickable index of the detailed notes. |
| `docs/MODEL_DETAILS.md` | Retained longer PAR6 appearance, jaw and authoring notes. |
| `docs/CHANGELOG.md` | Package version history. |
| `docs/DH.md` | Joint-angle convention and DH parameter definitions. |
| `docs/KINEMATICS_COMPARISON.md` | Numeric PAR6/PAROL6 differences and frame conventions. |
| `docs/KINEMATIC_REVIEW.md` | How to read the dimensioned review sheets and their validation. |
| `docs/DYNAMICS.md` | Applied masses/reference data and remaining estimates. |
| `docs/DYNAMICS_AUDIT.md` | RCB-versus-description mass-property comparison. |
| `docs/ACTUATION.md` | Simulation drives, torque limits and finite position-control assumptions. |
| `docs/GRASP_FRAMES.md` | Proposal for measured jaw-specific grasp frames; it does not mean those proposed frames have already been added. |
| `docs/CONTACT_PERFORMANCE.md` | Recorded timing comparison for refined gripper collision shapes. |
| `docs/SSG48_CONTACTS.md` | Diagnosis and correction of SSG48 rack/contact and grasp-fixture issues. |
| `docs/PNEUMATIC.md` | Pneumatic jaw travel, coupling and estimated physical/control data. |
| `docs/MIGRATION.md` | Compatibility and changes from earlier exports. |
| `docs/VALIDATION.md` | Standalone-gripper preservation checks, travel and known endpoint/contact limitations. |

For each of the six options, `docs/kinematics_review/<option>.svg` is the vector
dimensioned drawing, `.png` is its raster preview, and `.json` holds extracted
dimensions, DH rows and validation data. The drawings are generated from model
data; simulator code does not load them.

## Editable inputs and checked caches: `source/`

`source/COLCON_IGNORE` and `source/CATKIN_IGNORE` keep the preserved historical
ROS exports out of package discovery. The four maintained packages under
`urdf/` are the installable ROS products.

| Path | What it is |
| --- | --- |
| `artwork/par6_side_badge.svg` | Editable Figma artwork for the side-panel badge. |
| `appearance/colors.json` | Shared colors and part overrides for all eight packages, including the badge palette. Edit this to recolor models. |
| `artwork/par6_side_badge.json` | Badge size, placement and rendering configuration; historical colors are superseded by the shared palette during a complete build. |
| `artwork/texture_cache/par6_badge.png` | Checked rasterization of the SVG, reused to avoid platform-dependent antialiasing changes. |
| `artwork/texture_cache/par6_badge.json` | Input/generator hashes and cache metadata for that PNG. |
| `dynamics/physical_measurements.json` | User-reported measured masses: flange, complete grippers and moving assemblies. This does not provide newly measured COM/inertia for every part. |
| `dynamics/rcb_reference.json` | Versioned snapshot of the preferred RCB model: DH geometry, bounds, mass properties and captured estimates. Normal builds do not need RCB installed. |
| `dynamics/rcb_reference_checks.json` | Recorded RCB FK, gravity and mass-matrix samples used to test the portable reference implementation. |
| `dynamics/model_mapping.json` | Frame/body mapping for applying RCB properties to the CAD-derived models. |
| `dynamics/rcb_arm_reference.generated.json` | Generated arm-property conversion record; rebuild it rather than hand-editing it. |
| `grippers/previews/*.png` | Four standalone-gripper guide images, copied into exported packages and hardware guides. |
| `jaws/msg_tpu_glue/MSG_JAW_TPU_GLUE.STEP` | Original two-solid CAD source for the MSG Soft TPU jaw option. |
| `jaws/ssg48_pinch/solid_jaws_pinch.STEP` | Original CAD source for the default SSG48 Solid jaw option. |
| `jaws/*/jaw.json` | Jaw tessellation, placement, body assignment and generation settings; maintained colors come from the shared palette. |
| `jaws/*/tessellation_cache.json` | Checked CAD tessellation data and input/generator hashes. Allows unchanged CAD to reproduce the reviewed surface across platforms. |
| `parol6/original/` | Unmodified supplied PAROL6 URDF, ROS metadata and seven STL exports. These preserve the original geometry/frame evidence. |
| `parol6/model.json` | Source hashes, component/material assignments, geometry corrections, frame normalization and inheritance policy. |
| `parol6/README.md` | What is preserved from the source and what the generated PAROL6 changes. |
| `pneumatic/model.json` | Original L6 component assignments, mounting/travel settings and explicitly estimated pneumatic physical data. |
| `pneumatic/contact_cache/*.STL` | Checked convex contact meshes for the housing and two jaws. |
| `pneumatic/contact_cache/*.json` | Input signatures and piece counts for those contact meshes. |
| `reviews/render_cache/<robot>_<option>.png` | Twelve checked zero-pose robot snapshots used as artwork in dimensioned sheets. |
| `reviews/render_cache/<robot>_<option>.json` | Snapshot input hashes, camera and pixel-registration metadata. |
| `reviews/README.md` | Cache validation and explicit OpenGL refresh instructions. |

`source/legacy/` needs a little care: despite its name, **some of it is still
used by the generators** as original input and preservation evidence.

| Legacy subfolder | Contents |
| --- | --- |
| `legacy/assets/` | Original PAR6 meshes, native XML/scene and older SSG48 jaw sets. |
| `legacy/URDF/` | Original flange, MSG and SSG48 ROS exports: URDF, CSV mass/export tables, meshes, manifests and launch/config files. Also includes historical SSG48 jaw meshes. |
| `legacy/root/` | Earlier native MSG/SSG48 XML and scenes, preview images, package metadata, URDF notes and the original inertial-data PDF. |

These are provenance/build inputs, not the recommended packages to download.
Likewise, checked caches under `source/` are committed reproducibility inputs,
not disposable Python caches.

## Build and maintenance scripts: `tools/`

Most changes should be followed by `python tools/build.py`, as explained in
[BUILDING.md](BUILDING.md). Individual scripts are available for targeted work;
running every script manually is not the normal workflow.

| Script | Job |
| --- | --- |
| `build.py` | Main build: generates assets, synchronizes physical data, builds all products, validates them and writes ZIPs. |
| `split_motor_visuals.py` | Splits original triangle records into separately colored arm actuators and MSG lens/motor/structure visuals. |
| `model_colors.py` | Validates the shared palette, resolves overrides and applies visual colors without changing physical definitions. |
| `build_ros_support.py`, `ros_support/` | Native ROS packaging/launch templates, pure model-selection/resource helper and Linux integration validation. |
| `render_color_examples.py` | Renders the color guide's motor, jaw and badge comparisons from temporary model copies; needs OpenGL. |
| `build_side_badge.py` | Bakes SVG/palette onto the side-panel visual texture. |
| `build_msg_jaws.py` | Tessellates and installs the MSG Soft TPU jaw option. |
| `build_ssg48_jaws.py` | Tessellates and installs default SSG48 Solid jaws while retaining Fin-ray as an option. |
| `jaw_build_common.py` | Shared jaw-generation helpers. |
| `build_gripper_contacts.py` | Generates the convex finger collision pieces while preserving inertia. |
| `sync_model_dynamics.py` | Applies reference arm mass properties and corrected tool masses to the models. |
| `model_licensing.py` | Ships CC BY 4.0 model attribution and Apache-2.0 support-code notices in each package. |
| `model_reference.py` | Portable NumPy DH/dynamics implementation and repository paths, used instead of requiring the live RCB runtime. |
| `import_rcb_reference.py` | Explicitly imports a fresh snapshot from a specified RCB checkout. This optional operation needs that external checkout; the normal build does not. |
| `audit_rcb_dynamics.py` | Compares reference and model physical properties in consistent frames. |
| `build_parol6.py` | Generates PAROL6 from included original CAD exports and shared PAR6 tool/physical data. |
| `parol6_model.py` | Coordinate/frame helpers for the original PAROL6 chain and shared tools. |
| `build_pneumatic.py` | Splits the original pneumatic assembly into housing/two moving jaws and generates both formats. |
| `build_grasp_benches.py` | Creates the box/cylinder electric-gripper test fixtures. |
| `grasp_check.py` | Simulates close, lift, hold and release in those fixtures. |
| `build_kinematics_docs.py` | Writes common-frame/DH comparison and actuation documentation. |
| `build_review_sheets.py` | Builds twelve dimensioned sheets and their checks; optionally refreshes OpenGL snapshots or generates the PDF. |
| `build_standalone_grippers.py` | Exports independent MSG/SSG48 packages from maintained arm-mounted tool subtrees. |
| `render_msg_camera_previews.py` | Renders the actual named MSG camera in both existing box grasp fixtures, with recorded demo lighting and PNG/camera metadata. |
| `render_gripper_previews.py` | Optional OpenGL refresh of standalone guide images. MSG uses a temporary upright presentation pose, without changing saved mounting frames. |
| `render_parol6_previews.py` | Optional OpenGL refresh of PAROL6 preview images. |
| `benchmark_visuals.py` | Local timing comparison for rendering, physics and clearance queries. Timing records are measurements on a particular machine. |
| `ros_urdf.py` | Converts relative URDF mesh references to ROS package URIs. |
| `package_layout.py` | Shared docs/previews paths, documentation index and package-local link checks. |
| `package_models.py` | Creates all eight independent ZIPs, extracts them and checks their contents/assets. |
| `prepare_release.py` | Checks version consistency and prepares release assets, manifest and SHA-256 checksums. |
| `export_hardware_guide.py` | Copies a product guide and its local previews to a hardware repository, linking model/docs paths to a chosen central revision. |
| `verify_dynamics_sync.py` | Checks applied reference dynamics and measured masses; can also compare against a supplied older package. |
| `verify_motor_visuals.py` | Checks visual partition preservation and unchanged mechanics against supplied earlier packages. |
| `verify_msg_jaws.py` | Checks MSG jaw geometry, model definitions, coupling and retained physics. |
| `verify_ssg48_jaws.py` | Checks default Solid jaws and the retained Fin-ray option. |
| `verify_gripper_contacts.py` | Compares contact coverage and mechanics with supplied pre-contact packages. |
| `verify_ssg48_contacts.py` | Sweeps both jaw options for unintended SSG48 internal contacts on both arms. |
| `verify_parol6.py` | Checks PAROL6 frames, source-chain correspondence, shared physical data and tool transforms. |
| `verify_standalone_grippers.py` | Compares standalone and mounted grippers, runs close/open cycles and checks relocated packages. |

The JSON files next to the scripts are records, not additional executables:

| File(s) | Purpose |
| --- | --- |
| `motor_visual_partitions.json` | Description of the generated motor/structure mesh partitions. |
| `motor_visual_validation.json` | Recorded triangle/mechanics preservation results. |
| `msg_jaw_geometry.json`, `ssg48_jaw_geometry.json` | CAD hashes, tessellation/placement and generated-jaw geometry records. |
| `msg_jaw_validation.json`, `ssg48_jaw_validation.json` | Recorded jaw/model preservation and closing checks. |
| `side_badge_geometry.json` | Badge geometry, texture layout and panel containment record. |
| `visual_performance.json`, `msg_jaw_performance.json` | Recorded local appearance/jaw timing comparisons. |
| `side_badge_performance.json`, `side_badge_optimization_performance.json` | Recorded badge timing before/after optimization. |

Several validation/timing records document earlier milestones. They are not a
promise that every measurement was rerun on the current machine or release.
The `verify_*` commands documented as baseline comparisons require the named
older packages; those packages are not required for the normal clean build.

## Automated tests: `tests/`

| File | Coverage |
| --- | --- |
| `conftest.py` | Shared test setup/import paths. |
| `test_package_layout.py` | All eight packages have portable documentation/catalog links and the organized layout. |
| `test_licensing.py` | All eight packages have consistent model-license and support-code notices. |
| `test_models.py` | Measured masses and COM/inertia coordinate conversion. |
| `test_reference.py` | Captured-reference FK/dynamics regression samples. |
| `test_parol6.py` | PAROL6 geometry, frames and shared model policy. |
| `test_pneumatic.py` | Pneumatic component generation, coupling and model data. |
| `test_standalone_grippers.py` | Gripper-only exports and equivalence to mounted tools. |
| `test_gripper_contacts.py` | Grasp fixture/contact regression checks. |
| `test_review_sheets.py` | Dimensioned sheets, DH checks and snapshot validation. |
| `test_badge_cache.py` | Badge raster-cache reuse/invalidation. |
| `test_model_colors.py` | Non-default themes across every model, override/badge handling, invalid settings and unchanged compiled physical arrays. |
| `test_ros_support.py` | All sixteen model selections, resource URI conversion without model edits, compatibility aliases and invalid choices/assets. |
| `test_tessellation_cache.py` | Jaw CAD-cache reuse/invalidation. |
| `test_portability.py` | Authoring tools import without application/runtime dependencies. |
| `test_release.py` | Release versions, archive safety and generated release metadata. |

The full suite currently expands into 238 test cases. ROS 2 CI additionally runs
24 installed launch/TF/GUI cases per distribution. Simulation checks protect
model behavior; they do not turn estimated material/contact parameters into
measured hardware values.

## Review results and history: `reports/`

| File(s) | What they contain |
| --- | --- |
| `rcb_model_comparison.md` / `.json` | Human-readable and structured reference-versus-model mass-property audit. |
| `dynamics_alignment_plan.md` | Plan that guided the dynamics alignment work. |
| `dynamics_sync_validation.md` / `.json` | Applied measured masses/reference data and recorded synchronization results. The Markdown records the original alignment milestone. |
| `grasp_frames.md` | Proposal for adding separately measured jaw contact/grasp frames. |
| `grasp_checks.json` | Latest generated results for 16 arm/profile/object grasp cases. |
| `gripper_contact_geometry.json` | Generated convex contact decomposition details. |
| `gripper_contact_validation.json` | Contact coverage and preservation validation record. |
| `gripper_contact_performance.md` / `.json` | Recorded local timing check for the collision refinement. |
| `ssg48_contact_diagnosis.md` | Explanation of transmission-rack collisions and the corrected short-jaw grasp fixture. |
| `ssg48_internal_contacts.json` | Generated contact sweeps over SSG48 opening ranges. |
| `ssg48_before_after_validation.json` | Recorded comparison with the earlier contact geometry. |
| `par6_parol6_kinematics.md` | Common DH convention, numeric dimensions, axes and wrist differences. |
| `kinematics_comparison.svg` / `.png` | Frame/dimension comparison drawing and raster preview. |
| `parol6_build.json` | Generated PAROL6 build/mapping record. |
| `parol6_validation.json` | Source-chain, shared-property and tool-transform checks. |
| `parol6_actuation_smoke.json` | Recorded finite-controller simulation smoke check. |
| `parol6_models.png` | Gallery of all six PAROL6 tool configurations. |
| `parol6_wrist_mounting.png` | Wrist/tool-mounting comparison illustration. |
| `parol6_build_validation.md` | Earlier PAR6/PAROL6 build/archive validation milestone. |
| `parol6_frame_normalization_validation.md` | Earlier common-axis/frame-normalization validation milestone. |
| `pneumatic_geometry.json` | Pneumatic mesh partitions, contacts, bounds and estimated mass distribution. |
| `standalone_grippers_validation.json` | Generated gripper-only FK, inertia, contact and close/open checks. |
| `standalone_build_validation.md` | Historical clean-checkout authoring validation milestone, predating gripper-only exports. |
| `package_validation.json` | Latest generated ZIP extraction, entry-file, asset and mass checks. |
| `verification.json` | Earlier general model-preservation verification record. |

`reports/kinematics_review/` contains:

| File(s) | Purpose |
| --- | --- |
| `README.md` | How to read the twelve dimensioned review sheets. |
| `index.html` | Offline comparison page for browsing sheets side by side. |
| `validation.json` | Generated DH/FK and drawing checks. |
| `model_definition_baseline.json` | Model-definition hash record used for the review milestone. |
| `build_validation.md` | Recorded pneumatic/review build milestone. |
| `2.10.0_build_validation.md`, `2.11.0_build_validation.md` | Historical validation notes for those releases. |
| `2.10.0_model_definition_baseline.json`, `2.11.0_model_definition_baseline.json` | Model-definition hashes for those releases. |

These records are useful evidence, but read their recorded version/scope before
treating a historical result as a current release result. Current GitHub CI
runs and release manifests identify the built revision.

## PDF, release notes and local release output

- [`output/pdf/robot_kinematics_review.pdf`](output/pdf/robot_kinematics_review.pdf)
  is the twelve-page booklet: PAR6 and PAROL6, each with six tool configurations.
- `releases/v*.md` are versioned release-note sources, including the shared palette in `v2.16.0.md` and illustrated examples in `v2.16.1.md`.
- A local build produces eight ZIPs under `dist/`. `prepare_release.py` adds the
  release PDF, `release_manifest.json` and `SHA256SUMS`.
- The manifest describes the packaged revision/assets; checksums verify the
  downloaded bytes. GitHub Releases also offers an automatic source-code ZIP,
  which contains the authoring repository rather than a single runtime package.

## Where to make a change

| Change | Start with |
| --- | --- |
| Use a robot/gripper | `guides/<product>.md`, then one runtime package or release ZIP. |
| Edit a measured mass | `source/dynamics/physical_measurements.json`, then rebuild and validate. |
| Refresh preferred arm reference data | Explicit `tools/import_rcb_reference.py` import, then review and rebuild. |
| Change model or badge colors | `source/appearance/colors.json`. |
| Change the side logo shape/placement | `source/artwork/par6_side_badge.svg` and `.json`. |
| Change STEP jaw geometry | The relevant `source/jaws/` STEP and `jaw.json`. |
| Change motor/lens visual partition assignments | `tools/split_motor_visuals.py`; PAROL6-specific source assignments are in `source/parol6/model.json`. |
| Change PAROL6 normalization/mounting policy | `source/parol6/model.json` and its generator, followed by frame/geometry review. |
| Change a preview pose | The relevant `render_*` tool, then refresh the PNGs and export guides. |
| Change a robot link/joint definition | Review the tracked XML/URDF and the generators that update that field; see `BUILDING.md`. Not all robot structure is regenerated from CAD. |

For ordinary use, download one package. For maintenance, keep the source inputs
and generators: manually editing a generated mesh/report may be overwritten by
the next build. Historical records could be reorganized later, but deleting
`source/legacy/` or checked caches without updating the build would break the
current authoring workflow.
